#include "services/adsb_client.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include <ArduinoJson.h>

#include <algorithm>
#include <cstring>

#include "config.h"
#include "services/aircraft_motion.h"

namespace services::adsb {

namespace {

constexpr char kApiBase[] = "https://opendata.adsb.fi/api/v3/lat/";
constexpr float kKmPerNm = 1.852f;
constexpr int kConnectAttemptMs = 200;
constexpr unsigned long kRequestTimeoutMs = 20000;

using motion::AircraftTrack;

AircraftTrack s_tracks[kMaxAircraft];
AircraftTrack s_next_tracks[kMaxAircraft];
Aircraft s_incoming[kMaxAircraft];
Aircraft s_display[kMaxAircraft];
size_t s_aircraft_count = 0;
PollFn s_poll_fn = nullptr;
// Failure backoff: after a failed fetch (TLS malloc failure, HTTP error,
// parse error), skip further fetch attempts for this long. Without this, a
// memory-exhausted TLS stack retries every ~130 ms (main loop delay(10) +
// interval reset on failure) and hammers connect() forever.
unsigned long s_last_fetch_failure_ms = 0;
constexpr unsigned long kFetchFailureBackoffMs = 30000;
// Track coast time: how long a previously-seen aircraft stays on screen after
// its last contact. The ADS-B server/Cloudflare frequently truncates or empties
// wide-radius responses, which used to make aircraft vanish and reappear every
// fetch. Coasting keeps them displayed (position keeps extrapolating from last
// known velocity) until they've been absent this long.
constexpr unsigned long kTrackCoastMs = 45000;
portMUX_TYPE s_tracks_lock = portMUX_INITIALIZER_UNLOCKED;

bool fetchOnBackoff(unsigned long now_ms) {
  if (s_last_fetch_failure_ms == 0) {
    return false;
  }
  return (now_ms - s_last_fetch_failure_ms) < kFetchFailureBackoffMs;
}

void markFetchFailure(unsigned long now_ms) {
  s_last_fetch_failure_ms = now_ms;
}

int findTrackById(const char* id) {
  if (id == nullptr || id[0] == '\0') {
    return -1;
  }
  for (size_t i = 0; i < s_aircraft_count; ++i) {
    if (strcmp(s_tracks[i].aircraft.id, id) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void replaceTracks(const Aircraft* incoming, size_t count,
                   unsigned long now_ms) {
  taskENTER_CRITICAL(&s_tracks_lock);

  // Track which existing entries were refreshed this cycle, so stale ones can
  // be coasted (kept on screen) instead of dropped.
  bool refreshed[kMaxAircraft] = {false};

  // 1. Update/create tracks from the incoming response.
  size_t out = 0;
  for (size_t i = 0; i < count; ++i) {
    const int previous = motion::hasStableId(incoming[i])
                             ? findTrackById(incoming[i].id)
                             : -1;
    if (previous >= 0) {
      refreshed[previous] = true;
    }
    s_next_tracks[out++] =
        previous >= 0
            ? motion::updateTrack(s_tracks[previous], incoming[i], now_ms)
            : motion::makeInitialTrack(incoming[i], now_ms);
  }

  // 2. Coast: carry over tracks that were refreshed recently but are missing
  // from this response (truncated / empty / server flake). Their positions
  // keep extrapolating from last known velocity until they've been absent
  // longer than kTrackCoastMs. This stops the wide view from blanking out and
  // re-populating every fetch.
  for (size_t i = 0; i < s_aircraft_count && out < kMaxAircraft; ++i) {
    if (!refreshed[i] && motion::hasStableId(s_tracks[i].aircraft) &&
        motion::elapsedMs(now_ms, s_tracks[i].updated_ms) < kTrackCoastMs) {
      s_next_tracks[out++] = s_tracks[i];
    }
  }

  for (size_t i = 0; i < out; ++i) {
    s_tracks[i] = s_next_tracks[i];
  }
  s_aircraft_count = out;
  taskEXIT_CRITICAL(&s_tracks_lock);
}

void pollNetwork() {
  if (s_poll_fn != nullptr) {
    s_poll_fn();
  }
}

int performGetWithPoll(HTTPClient& http) {
  http.setConnectTimeout(kConnectAttemptMs);
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  int attempts = 0;
  constexpr int kMaxConnectAttempts = 3;
  while (millis() < deadline && attempts < kMaxConnectAttempts) {
    ++attempts;
    pollNetwork();
    const int code = http.GET();
    if (code > 0) {
      return code;
    }
    if (code != HTTPC_ERROR_CONNECTION_REFUSED &&
        code != HTTPC_ERROR_NOT_CONNECTED && code != -1) {
      return code;
    }
    delay(5);
  }
  return HTTPC_ERROR_READ_TIMEOUT;
}

bool readResponseBodyWithPoll(HTTPClient& http, String& payload) {
  WiFiClient* stream = http.getStreamPtr();
  if (stream == nullptr) {
    return false;
  }

  payload.reserve(4096);
  const unsigned long deadline = millis() + kRequestTimeoutMs;

  // Content-Length responses: read exactly getSize() bytes (definitive).
  const int content_length = http.getSize();
  if (content_length > 0) {
    uint8_t buffer[512];
    while (millis() < deadline &&
           static_cast<int>(payload.length()) < content_length) {
      pollNetwork();
      const int available = stream->available();
      if (available > 0) {
        const int to_read =
            available > static_cast<int>(sizeof(buffer)) ? static_cast<int>(sizeof(buffer))
                                                         : available;
        const int read_bytes = stream->readBytes(buffer, to_read);
        if (read_bytes > 0) {
          payload.concat(reinterpret_cast<const char*>(buffer),
                         static_cast<unsigned>(read_bytes));
        }
      } else if (!http.connected()) {
        break;
      }
      delay(1);
    }
    return payload.length() > 0;
  }

  // Chunked Transfer-Encoding (getSize() == -1 on HTTP/1.1). Decode chunks
  // properly: read hex-size line -> that many bytes -> CRLF; stop at a
  // zero-size chunk (the definitive end marker). No silence guessing.
  while (millis() < deadline) {
    pollNetwork();
    const String size_line = stream->readStringUntil('\n');
    if (size_line.length() == 0) {
      if (!http.connected()) {
        break;
      }
      delay(1);
      continue;
    }
    const int chunk_size =
        static_cast<int>(strtol(size_line.c_str(), nullptr, 16));
    if (chunk_size == 0) {
      break;  // terminal chunk
    }
    uint8_t buffer[512];
    int remaining = chunk_size;
    while (remaining > 0 && millis() < deadline) {
      pollNetwork();
      const int to_read =
          remaining > static_cast<int>(sizeof(buffer)) ? static_cast<int>(sizeof(buffer))
                                                       : remaining;
      const int read_bytes = stream->readBytes(buffer, to_read);
      if (read_bytes > 0) {
        payload.concat(reinterpret_cast<const char*>(buffer),
                       static_cast<unsigned>(read_bytes));
        remaining -= read_bytes;
      } else if (!http.connected()) {
        break;
      }
      delay(1);
    }
    // Consume the CRLF after each chunk (may be split across reads; tolerate
    // failure via readStringUntil on the next iteration).
    stream->readStringUntil('\n');
  }

  return payload.length() > 0;
}

float kmToNauticalMiles(float km) { return km / kKmPerNm; }

bool readJsonFloat(const JsonObject& obj, const char* key, float* out) {
  if (obj[key].is<float>() || obj[key].is<double>() || obj[key].is<int>()) {
    *out = obj[key].as<float>();
    return true;
  }
  return false;
}

float pickNoseHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickTrackHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickGroundSpeed(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "gs", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "tas", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "ias", &v)) {
    return v;
  }
  return 0.0f;
}

bool isOnGround(const JsonObject& plane) {
  if (!plane["alt_baro"].is<const char*>()) {
    return false;
  }
  return strcmp(plane["alt_baro"].as<const char*>(), "ground") == 0;
}

void copyJsonStringTrimmed(const JsonObject& obj, const char* key, char* out,
                           size_t out_len) {
  out[0] = '\0';
  if (out_len == 0 || !obj[key].is<const char*>()) {
    return;
  }
  const char* s = obj[key].as<const char*>();
  size_t n = strnlen(s, out_len - 1);
  while (n > 0 && s[n - 1] == ' ') {
    --n;
  }
  memcpy(out, s, n);
  out[n] = '\0';
}

void formatAltitudeTag(const JsonObject& plane, char* out, size_t out_len) {
  out[0] = '\0';
  if (out_len == 0) {
    return;
  }

  if (plane["alt_baro"].is<const char*>()) {
    const char* s = plane["alt_baro"].as<const char*>();
    if (strcmp(s, "ground") == 0) {
      strncpy(out, "GND", out_len - 1);
      out[out_len - 1] = '\0';
      return;
    }
  }

  float alt = 0.0f;
  if (readJsonFloat(plane, "alt_baro", &alt) ||
      readJsonFloat(plane, "alt_geom", &alt)) {
    snprintf(out, out_len, "%d ft", static_cast<int>(lroundf(alt)));
  }
}

void fillTagFields(Aircraft* ac, const JsonObject& plane) {
  copyJsonStringTrimmed(plane, "hex", ac->id, sizeof(ac->id));
  copyJsonStringTrimmed(plane, "flight", ac->callsign, sizeof(ac->callsign));
  if (ac->callsign[0] == '\0') {
    strncpy(ac->callsign, ac->id, sizeof(ac->callsign) - 1);
    ac->callsign[sizeof(ac->callsign) - 1] = '\0';
  }

  copyJsonStringTrimmed(plane, "t", ac->type, sizeof(ac->type));
  formatAltitudeTag(plane, ac->alt, sizeof(ac->alt));
}

}  // namespace

void setPollFn(PollFn fn) { s_poll_fn = fn; }

size_t aircraftCount() {
  taskENTER_CRITICAL(&s_tracks_lock);
  const size_t count = s_aircraft_count;
  taskEXIT_CRITICAL(&s_tracks_lock);
  return count;
}

const Aircraft* aircraftList(unsigned long now_ms, size_t* count) {
  taskENTER_CRITICAL(&s_tracks_lock);
  const size_t display_count = s_aircraft_count;
  for (size_t i = 0; i < display_count; ++i) {
    s_display[i] = s_tracks[i].aircraft;
    const motion::Position position =
        motion::displayPosition(s_tracks[i], now_ms);
    s_display[i].lat = position.lat;
    s_display[i].lon = position.lon;
  }
  taskEXIT_CRITICAL(&s_tracks_lock);
  if (count != nullptr) {
    *count = display_count;
  }
  return s_display;
}

bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km) {
  const unsigned long now_ms = millis();
  if (fetchOnBackoff(now_ms)) {
    // Keep showing last known planes; don't hammer TLS while heap is tight.
    return false;
  }

  const float dist_nm = kmToNauticalMiles(fetch_radius_km);

  String url = kApiBase;
  url += String(center_lat, 6);
  url += "/lon/";
  url += String(center_lon, 6);
  url += "/dist/";
  url += String(dist_nm, 1);

  // PERSISTENT TLS connection: build the client+HTTPClient ONCE and reuse
  // across every fetch. Creating fresh objects per fetch re-handshakes TLS
  // each time and leaks ~13 KB/cycle (observed heap 73740 -> 60016 -> 55992),
  // until the next handshake fails with -32512 (SSL malloc). One connection,
  // one handshake, HTTP/1.1 keep-alive, zero accumulation.
  static WiFiClientSecure client;
  static HTTPClient http;
  static bool s_client_ready = false;
  if (!s_client_ready) {
    client.setInsecure();
    http.setReuse(true);  // keep the TLS socket open between fetches
    s_client_ready = true;
  }

  if (!http.begin(client, url)) {
    Serial.println("adsb: http.begin failed");
    return false;
  }
  // Ask for plain JSON (avoids gzip; the chunked decoder handles the rest).
  http.addHeader("Accept-Encoding", "identity");

  http.setTimeout(kRequestTimeoutMs);
  const int code = performGetWithPoll(http);
  if (code != HTTP_CODE_OK) {
    Serial.printf("adsb: HTTP %d\n", code);
    markFetchFailure(millis());
    // Connection broken; tear down so the next cycle rebuilds it fresh.
    http.end();
    client.stop();
    s_client_ready = false;
    return false;
  }

  // Read the body: Content-Length or chunked-decoded, with pollNetwork() so
  // large TLS bodies keep flowing. Returns the COMPLETE body (the old
  // truncations were the TLS re-buffer pauses + missing chunked decoder).
  String payload;
  if (!readResponseBodyWithPoll(http, payload)) {
    // Empty body (server flake / Cloudflare hiccup) is not a network failure -
    // keep the last good frame and retry next cycle.
    Serial.println("adsb: empty response - keeping last frame");
    http.end();
    client.stop();
    s_client_ready = false;
    return false;
  }
  // Keep the connection alive for the next fetch (do NOT end()).

  // Parse the JSON window between the first '{' and last '}'. The raw body
  // from the reader is clean JSON (Cloudflare's chunked framing is decoded by
  // the TLS/read layer); do NOT mutate the String in place — Arduino String
  // operator[] is not guaranteed writable, and memmove into it corrupted the
  // buffer (InvalidInput on valid JSON). No extra RAM, no copying.
  const int json_start = payload.indexOf('{');
  const int json_end = payload.lastIndexOf('}');
  const char* json_ptr = json_start < 0 ? payload.c_str() : payload.c_str() + json_start;
  const size_t json_len =
      (json_start < 0 || json_end < json_start)
          ? 0
          : static_cast<size_t>(json_end - json_start + 1);

  // Use a fixed-size static JSON document. A `static DynamicJsonDocument`
  // inside this function still heap-allocates its 16 KB on FIRST use — which
  // is right in the middle of fetch 1, fragmenting the heap before fetch 2's
  // TLS handshake (observed: -32512 after the first successful fetch). A
  // namespace-scope StaticJsonDocument lives in BSS (allocated at boot,
  // zero heap cost, never fragments). 8 KB fits the ~7 KB capped-radius
  // payload; aircraft beyond capacity are not rendered.
  static StaticJsonDocument<8 * 1024> doc;
  const DeserializationError err =
      deserializeJson(doc, json_ptr, json_len);
  // Only a COMPLETE, valid response updates the display. Anything else
  // (EmptyInput, IncompleteInput, InvalidInput, NoMemory) means the server
  // hiccuped or truncated mid-body — swapping in a partial subset made the
  // wide view erratic (planes randomly appearing/disappearing each 3 s fetch).
  // Keep the last good frame instead; it stays stable until a full response
  // arrives.
  if (err != DeserializationError::Ok) {
    // Truncated/incomplete is NOT a network failure — keep the last good frame
    // and retry on the next fetch cycle (no backoff), so the wide view stays
    // stable but still refreshes whenever a complete response arrives.
    Serial.printf("adsb: fetch incomplete (%s) - len=%u head='%.*s' tail='%.*s'\n",
                  err.c_str(), static_cast<unsigned>(json_len), json_len > 24 ? 24 : static_cast<int>(json_len),
                  json_ptr, 28,
                  json_ptr + (json_len > 28 ? json_len - 28 : 0));
    return false;
  }

  JsonArray ac = doc["ac"].as<JsonArray>();
  if (ac.isNull()) {
    // Complete, valid response with no "ac" key / empty array. With track
    // coasting this does NOT wipe the screen — existing tracks stay displayed
    // (extrapolated) until they've been absent for kTrackCoastMs. A
    // server-flaked empty response no longer blanks the radar.
    replaceTracks(nullptr, 0, millis());
    return true;
  }

  size_t n = 0;
  for (JsonObject plane : ac) {
    if (n >= kMaxAircraft) {
      break;
    }
    if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) {
      continue;
    }
    if (isOnGround(plane) && !config::kAdsbShowGroundAircraft) {
      continue;
    }

    s_incoming[n].lat = plane["lat"].as<float>();
    s_incoming[n].lon = plane["lon"].as<float>();
    s_incoming[n].nose_deg = pickNoseHeading(plane);
    s_incoming[n].track_deg = pickTrackHeading(plane);
    s_incoming[n].gs_knots = pickGroundSpeed(plane);
    fillTagFields(&s_incoming[n], plane);
    ++n;
  }

  replaceTracks(s_incoming, n, millis());
  Serial.printf("adsb: %u aircraft\n", static_cast<unsigned>(n));
  return true;
}

}  // namespace services::adsb
