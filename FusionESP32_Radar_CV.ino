/**
 * ============================================================
 *  Fusion ESP32 — Radar + Computer Vision Object Detection
 * ============================================================
 *
 *  Architecture:
 *    PC (Python YOLO) ──TCP "ON\n"──► This ESP32 (WiFi TCP server)
 *    RD03D Radar      ──Serial2────► This ESP32
 *                                         │
 *                              GPIO triggers relay/LED/buzzer
 *                              only when BOTH sensors agree
 *
 *  Wiring:
 *    RD03D Pin 1 (5V)  → ESP32 5V
 *    RD03D Pin 2 (GND) → ESP32 GND
 *    RD03D Pin 3 (TX)  → ESP32 GPIO32
 *    RD03D Pin 4 (RX)  → ESP32 GPIO33
 *    Output device     → OUTPUT_PIN (default GPIO2)
 *
 *  This ESP32 must use the same IP as ESP32_LED_IP in yolo_tcp.py
 *  Either assign a static IP below, or reserve the IP in your router.
 * ============================================================
 */

#include <WiFi.h>

// ─── WiFi credentials ────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";      // ← change me
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";  // ← change me

// ─── TCP server (must match TCP_PORT in yolo_tcp.py) ─────────
const uint16_t TCP_PORT = 4210;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// ─── Radar Serial pins ───────────────────────────────────────
#define RADAR_RX_PIN   32     // Connected to RD03D TX
#define RADAR_TX_PIN   33     // Connected to RD03D RX
#define RADAR_BAUD     256000

// ─── Output GPIO (relay / LED / buzzer) ──────────────────────
#define OUTPUT_PIN     2      // ← change to your pin

// ─── RD03D Frame definition ──────────────────────────────────
//  Byte index:  0    1    2    3   = header
//               4-5  = X coord (little-endian 16-bit)
//               6-7  = Y coord
//               8-9  = Velocity
//               10-11= Distance
//               ...padding...
//               29   = 0x55  (footer)
//               30   = 0xCC  (footer)
#define FRAME_LEN 31
const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t FOOTER[2] = {0x55, 0xCC};

// ─── Detection state ─────────────────────────────────────────
volatile bool cvDetected    = false;
volatile bool radarDetected = false;
unsigned long cvLastTime    = 0;
unsigned long radarLastTime = 0;

// CV signal expires if no "ON" received for this long.
// Matches the 15s LED timeout comment in yolo_tcp.py.
const unsigned long CV_TIMEOUT_MS    = 15000;

// Radar clears if no valid in-range frame received for this long.
const unsigned long RADAR_TIMEOUT_MS = 3000;

//*************ADJUST THESE VARIABLES***************************//
// ─── Radar detection thresholds ──────────────────────────────
// Adjust RADAR_DIST_MAX to match your physical detection zone.
const uint16_t RADAR_DIST_MIN_CM = 10;   // ignore noise below
const uint16_t RADAR_DIST_MAX_CM = 500;  // ~5 m max range

// ─── Radar frame buffer ──────────────────────────────────────
uint8_t  radarBuf[FRAME_LEN];
uint8_t  radarIdx = 0;

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Fusion ESP32: Radar + CV Detection ===");

  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  // Start radar UART
  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  Serial.println("[INIT] Radar serial started on Serial2.");
  delay(2000);  // Let radar stabilise
  Serial.println("[INIT] Radar ready.");

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[INIT] Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\n[INIT] WiFi connected!");
  Serial.print("[INIT] IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("[INIT] Set this IP as ESP32_LED_IP in yolo_tcp.py");

  // Start TCP server
  tcpServer.begin();
  Serial.print("[INIT] TCP server listening on port ");
  Serial.println(TCP_PORT);

  Serial.println("\n=== System Ready ===\n");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  handleTCPClient();
  handleRadar();
  updateFusionOutput();
}

// ─────────────────────────────────────────────────────────────
//  TCP: Accept Python YOLO connection and read "ON\n" messages
// ─────────────────────────────────────────────────────────────
void handleTCPClient() {
  // Accept a new connection if none is active
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.accept();
    if (newClient) {
      tcpClient = newClient;
      Serial.println("[TCP]  Python YOLO client connected.");
    }
  }

  if (!tcpClient || !tcpClient.connected()) return;

  // Read any incoming data
  while (tcpClient.available()) {
    String line = tcpClient.readStringUntil('\n');
    line.trim();

    if (line == "ON") {
      cvDetected = true;
      cvLastTime = millis();
      // Python script reads this response to confirm delivery
      tcpClient.print("OK\n");
      Serial.println("[CV]   Pedestrian detected — TCP signal received.");
    }
  }

  // Expire CV detection after timeout (no new "ON" from Python)
  if (cvDetected && (millis() - cvLastTime > CV_TIMEOUT_MS)) {
    cvDetected = false;
    Serial.println("[CV]   Detection expired (timeout).");
  }
}

// ─────────────────────────────────────────────────────────────
//  Radar: Parse RD03D binary frames from Serial2
//
//  The parser uses a sliding-window approach to stay in sync —
//  if a byte doesn't match the expected header sequence it
//  resets, so partial or corrupt frames are safely discarded.
// ─────────────────────────────────────────────────────────────
void handleRadar() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    // ── Header sync (first 4 bytes must be AA FF 03 00) ──
    if (radarIdx < 4) {
      if (b == HEADER[radarIdx]) {
        radarBuf[radarIdx++] = b;
      } else {
        // Mismatch — reset, but check if this byte starts a new header
        radarIdx = 0;
        if (b == HEADER[0]) {
          radarBuf[radarIdx++] = b;
        }
      }
      continue;
    }

    // ── Accumulate payload + footer ──────────────────────
    radarBuf[radarIdx++] = b;

    if (radarIdx == FRAME_LEN) {
      radarIdx = 0;  // Ready for next frame

      // Validate footer
      if (radarBuf[29] != FOOTER[0] || radarBuf[30] != FOOTER[1]) {
        Serial.println("[RADAR] Bad footer — frame discarded.");
        return;
      }

      // Reconstruct 16-bit values (little-endian: low byte first)
      uint16_t xCoord   = ((uint16_t)radarBuf[5]  << 8) | radarBuf[4];
      uint16_t yCoord   = ((uint16_t)radarBuf[7]  << 8) | radarBuf[6];
      uint16_t velocity = ((uint16_t)radarBuf[9]  << 8) | radarBuf[8];
      uint16_t distance = ((uint16_t)radarBuf[11] << 8) | radarBuf[10];

      Serial.printf("[RADAR] X=%-5u  Y=%-5u  Vel=%-5u  Dist=%u cm\n",
                    xCoord, yCoord, velocity, distance);

      // Mark detected if distance is within the valid zone
      if (distance >= RADAR_DIST_MIN_CM && distance <= RADAR_DIST_MAX_CM) {
        radarDetected = true;
        radarLastTime = millis();
      }
    }
  }

  // Expire radar detection when frames stop reporting a target
  if (radarDetected && (millis() - radarLastTime > RADAR_TIMEOUT_MS)) {
    radarDetected = false;
    Serial.println("[RADAR] Detection expired (no target).");
  }
}

// ─────────────────────────────────────────────────────────────
//  Fusion: Drive output HIGH only when BOTH sensors agree.
//  This eliminates false positives from either sensor alone.
// ─────────────────────────────────────────────────────────────
void updateFusionOutput() {
  bool objectPresent = cvDetected && radarDetected;
  digitalWrite(OUTPUT_PIN, objectPresent ? HIGH : LOW);

  // Print state changes only (avoids spamming the serial monitor)
  static bool lastState = false;
  if (objectPresent != lastState) {
    lastState = objectPresent;
    Serial.println(objectPresent
      ? "[FUSION] ✓ Object confirmed by BOTH sensors → OUTPUT HIGH"
      : "[FUSION] ✗ Object gone / unconfirmed   → OUTPUT LOW");
    Serial.printf("         CV=%s  Radar=%s\n",
                  cvDetected    ? "YES" : "NO",
                  radarDetected ? "YES" : "NO");
  }
}
