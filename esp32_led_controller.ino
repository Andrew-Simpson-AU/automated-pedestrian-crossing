/**************************************************
 * ESP32 #2 - LED Controller Only
 * Receives commands from laptop to control LED
 * No camera - just listens for "ON" commands
 **************************************************/

#include <WiFi.h>

// LED Pin
#define LED_PIN 2  // IO2 is active-low: LOW=ON, HIGH=OFF

// WiFi credentials
const char* ssid = "The _house";
const char* password = "reasonauthor857";

// TCP Server
WiFiServer tcpServer(4210);
WiFiClient tcpClient;

// LED timer
unsigned long ledOnTime = 0;
bool ledState = false;
const unsigned long LED_TIMEOUT = 15000;  // 15 seconds

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 LED Controller ===");
  
  // Setup LED (active-low for pin 2: LOW=ON, HIGH=OFF)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Start OFF
  Serial.println("LED initialized (OFF)");
  
  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Start TCP server
  tcpServer.begin();
  Serial.print("TCP server listening on port 4210");
  Serial.println("\n=== READY - Waiting for commands ===");
}

void loop() {
  checkTCPClient();
  checkLEDTimer();
  delay(10);
}

void checkTCPClient() {
  // Accept new client
  if (!tcpClient || !tcpClient.connected()) {
    tcpClient = tcpServer.available();
    if (tcpClient) {
      Serial.println(">>> Client connected!");
      tcpClient.setNoDelay(true);
    }
  }
  
  // Read commands
  if (tcpClient && tcpClient.available()) {
    String command = tcpClient.readStringUntil('\n');
    command.trim();
    
    Serial.print("Received: '");
    Serial.print(command);
    Serial.println("'");
    
    if (command == "ON") {
      // Turn LED ON (active-low: LOW = ON)
      digitalWrite(LED_PIN, LOW);
      ledState = true;
      ledOnTime = millis();
      
      Serial.println(">>> LED ON (15s timer started)");
      
      // Send acknowledgment
      tcpClient.println("OK");
      tcpClient.flush();
    }
  }
}

void checkLEDTimer() {
  if (ledState && (millis() - ledOnTime > LED_TIMEOUT)) {
    digitalWrite(LED_PIN, HIGH);  // Turn OFF (active-low: HIGH = OFF)
    ledState = false;
    Serial.println(">>> LED OFF (timer expired)");
  }
}
