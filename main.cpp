#include <Arduino.h>
#include <RCSwitch.h>
#include <ArduinoJson.h> 
#include <WiFi.h>
#include <HTTPClient.h>
#include <mbedtls/md.h>

RCSwitch mySwitch = RCSwitch();

const char* ssid = "WSN_AP";
const char* password = "raspberry";
const int RF_PIN_RX = 21;  // ESP32 RX pin for 433MHz
const long EXPECTED_RF_CODE = 22;

const char* node_id = "wsn_07";
const char* mac_address = "80:B5:4E:C4:EF:8C";
const char* secret_key = "pi_secret_key_12345";
char session_id[64] = "";  // Fixed: Added size and initialized

String generateToken(const char* node_id, const char* metadata) {
    unsigned char output[32];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);  // Fixed: Removed corrupted text
    mbedtls_md_hmac_starts(&ctx, (const unsigned char*)secret_key, strlen(secret_key));

    String payload = String(node_id) + "|" + String(metadata);
    mbedtls_md_hmac_update(&ctx, (const unsigned char*)payload.c_str(), payload.length());
    mbedtls_md_hmac_finish(&ctx, output);
    mbedtls_md_free(&ctx);

    char hexstr[65];
    for (int i = 0; i < 32; i++) sprintf(hexstr + i*2, "%02x", output[i]);
    hexstr[64] = 0;
    return String(hexstr);
}

void connectAndOnboard() {
    Serial.println("Trigger received, connecting to AP...");
    String metadata = "lat=7.123;lon=80.456"; 

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to AP");

    String token = generateToken(node_id, metadata.c_str());

    HTTPClient http;
    http.begin("http://10.42.0.1:8080/onboard");
    http.addHeader("Content-Type", "application/json");
    String body = "{\"node_id\":\"" + String(node_id) + "\",\"mac\":\"" + String(mac_address) + "\",\"token\":\"" + token + "\",\"metadata\":\"" + metadata + "\"}";
    
    int httpResponseCode = http.POST(body);
    if (httpResponseCode == 200) {
        String res = http.getString();  // Fixed: Added semicolon
        Serial.println("Onboarding success: " + res);

        JsonDocument doc;  // Fixed: Changed from deprecated DynamicJsonDocument
        DeserializationError error = deserializeJson(doc, res);  // Fixed: Changed 'response' to 'res'
        if (!error && doc.containsKey("session_id")){  // Fixed: Corrected to containsKey (capital K)
            String sid = doc["session_id"].as<String>();
            sid.toCharArray(session_id, sizeof(session_id));
            Serial.println("Session ID stored: " + String(session_id));  // Fixed: Added semicolon

            http.begin("http://10.42.0.1:8080/ack");
            http.addHeader("Content-Type", "application/json");
            String ackBody = "{\"session_id\":\"" + String(session_id) + "\"}";  // Fixed: Proper string concatenation
            int ackResponseCode = http.POST(ackBody);
            if(ackResponseCode == 200){
                String ackResponse = http.getString();
                Serial.println("ACK response: " + ackResponse);
            }else {
                Serial.println("ACK failed: " + String(ackResponseCode));
            }
        }else {
            Serial.println("Failed to parse session_id from onboarding response");
        }
    } else {
        Serial.println("Onboarding failed: " + String(httpResponseCode));
    }
    http.end();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Ready to receive RF trigger");

    mySwitch.enableReceive(RF_PIN_RX);
}

void loop() {
    if (mySwitch.available()) {
        long value = mySwitch.getReceivedValue();
        if (value == EXPECTED_RF_CODE) {
            connectAndOnboard();
        }
        mySwitch.resetAvailable();
    }
}