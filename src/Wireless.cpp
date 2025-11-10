#include "Wireless.h"

JsonCommandCallback jsonCommandCallback = nullptr;
Wireless* g_pThis = nullptr;

#define WIFI_MODE_NONE 0
#define WIFI_MODE_AP_STA 1

JsonDocument jsonCmdReceiveEspnow;

typedef struct struct_message {
    char message[250];
  } struct_message;
struct_message espNowMessage;
struct_message espNowMegsRecv;
esp_now_peer_info_t peerInfo;

int wifiMode = 1;
int maxClients = 1;
bool statusAP = false;
bool statusSTA = false;

bool Wireless::setAP(String ssid, String password, int wifiChannel) {
    if (wifiMode == WIFI_MODE_NONE) {
        Serial.println("WiFi mode is None, skip configuring Access Point");
        Serial0.println("WiFi mode is None, skip configuring Access Point");
        return false;
    }
    if (ssid.length() == 0) {
        Serial.println("SSID is empty, skip configuring Access Point");
        Serial0.println("SSID is empty, skip configuring Access Point");
        return false;
    }
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ssid.c_str(), password.c_str(), wifiChannel, 0, maxClients);
    if (WiFi.softAPIP()) {
        Serial.println("Access Point started");
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial0.println("Access Point started");
        Serial0.print("IP Address: ");
        Serial0.println(WiFi.softAPIP());
        return true;
    } else {
        Serial.println("Failed to start Access Point");
        Serial0.println("Failed to start Access Point");
        return false;
    }
}

bool Wireless::setSTA(String ssid, String password) {
    if (wifiMode == WIFI_MODE_NONE) {
        Serial.println("WiFi mode is None, skip configuring Station");
        Serial0.println("WiFi mode is None, skip configuring Station");
        return false;
    }
    if (ssid.length() == 0) {
        Serial.println("SSID is empty, skip configuring Station");
        Serial0.println("SSID is empty, skip configuring Station");
        return false;
    }
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi");
    Serial0.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        Serial0.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial0.println("\nWiFi connected");
        Serial0.print("IP Address: ");
        Serial0.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nFailed to connect to WiFi");
        Serial0.println("\nFailed to connect to WiFi");
        return false;
    }
}

bool Wireless::setWifiMode(int mode, String ap_ssid, String ap_password, int wifiChannel, String sta_ssid, String sta_password) {
    if(mode == WIFI_MODE_NONE) {
        WiFi.mode(WIFI_OFF);
        wifiMode = WIFI_MODE_NONE;
        return false;
    } else if (mode == WIFI_MODE_AP_STA) {
        WiFi.mode(WIFI_AP_STA);
        wifiMode = WIFI_MODE_AP_STA;
        bool result = WiFi.softAP(ap_ssid.c_str(), ap_password.c_str(), wifiChannel);
        Serial.println(result ? "AP started!" : "AP start failed!");
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        if(setSTA(sta_ssid.c_str(), sta_password.c_str())) {
            Serial.print("STA IP address: ");
            Serial.println(WiFi.localIP().toString().c_str());
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

int Wireless::getRSSI_STA() {
    if (WiFi.status() == WL_CONNECTED) {
        int rssi = WiFi.RSSI();
        return rssi;
    } else {
        return 1;
    }
}

int Wireless::getRSSI_AP() {
    wifi_sta_list_t stationList;
    esp_wifi_ap_get_sta_list(&stationList);
    wifi_sta_info_t station = stationList.sta[0];
    return station.rssi;
}

String Wireless::getAPIP() {
    if (WiFi.softAPIP()) {
        return WiFi.softAPIP().toString();
    } else {
        return "";
    }
}

String Wireless::getSTAIP() {
    if (WiFi.localIP()) {
        return WiFi.localIP().toString();
    } else {
        return "";
    }
}



void Wireless::printKnownMacs() {
    Serial.println("--- Known MAC Addresses ---");
    if (knownMacs.empty()) {
        Serial.println("Empty");
        return;
    }
    for (size_t i = 0; i < knownMacs.size(); ++i) {
        Serial.print("Index ");
        Serial.print(i);
        Serial.print(": ");
        for (int j = 0; j < 6; ++j) {
            Serial.printf("%02X", knownMacs[i][j]);
            if (j < 5) Serial.print(":");
        }
        Serial.println();
    }
    Serial.println("--------------------------");
}

bool Wireless::isKnownMac(const uint8_t *mac) {
    for (const auto& knownMac : knownMacs) {
        if (memcmp(knownMac.data(), mac, 6) == 0) {
            return true;
        }
    }
    return false;
}

void Wireless::OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (espnowMode == 0) {
        // do nothing
        return;
    }
    if (espnowMode == 1) {
        if (!isKnownMac(mac_addr)) {
            // do nothing
            return;
        }
    }
    if (data_len == sizeof(struct_message)) {
        memcpy(&espNowMegsRecv, data, sizeof(espNowMegsRecv));
        espNowMegsRecv.message[sizeof(espNowMegsRecv.message) - 1] = '\0';
        jsonCommandCallback(espNowMegsRecv.message);
        Serial.println(espNowMegsRecv.message);
    }
} 

void IRAM_ATTR Wireless::staticRecvCallback(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (g_pThis) {
        g_pThis->OnDataRecv(info->src_addr, incomingData, len);
    }
}

void Wireless::espnowInit(bool longRange) {
    if (longRange) {
        if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) == ESP_OK) {
            Serial.println("Long Range Mode enabled for STA");
        } else {
            Serial.println("Failed to enable Long Range Mode for STA");
        }

        if (esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR) == ESP_OK) {
            Serial.println("Long Range Mode enabled for AP");
        } else {
            Serial.println("Failed to enable Long Range Mode for AP");
        }
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    g_pThis = this;
    esp_now_register_recv_cb(staticRecvCallback);

    // example for adding known MACs
    // MacAddress defaultMac1 = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};
    // MacAddress defaultMac2 = {0x24, 0x6F, 0x28, 0x12, 0x34, 0x56};

    // knownMacs.push_back(defaultMac1);
    // knownMacs.push_back(defaultMac2);
}



bool Wireless::setEspNowMode(int mode) {
    if (mode == 0) {
        espnowMode = 0;
        return true;
    } else if (mode == 1) {
        espnowMode = 1;
        return true;
    } else if (mode == 2) {
        espnowMode = 2;
        return true;
    } else {
        return false;
    }
}

String Wireless::macToString(uint8_t mac[6]) {
    char macStr[18]; // 6 pairs of 2 characters + null terminator
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

String Wireless::getMac() {
    if (fakeMac) {
        return "FF:FF:FF:FF:FF:FF";
    } else {
        return WiFi.macAddress();
    }
}

void Wireless::macStringToByteArray(const String& macString, uint8_t* byteArray) {
    for (int i = 0; i < 6; i++) {
      byteArray[i] = strtol(macString.substring(i * 3, i * 3 + 2).c_str(), NULL, 16);
    }
    return;
}

bool Wireless::sendEspNow(String macInput, String data) {
    if (macInput.length() != 17) {
        Serial.println("invalid MAC address format.");
        return false;
    }

    if (data.length() >= sizeof(struct_message::message)) {
        Serial.println("Data too long for struct message buffer.");
        return false;
    }

    uint8_t macArray[6];
    macStringToByteArray(macInput, macArray);

    strcpy(espNowMessage.message, data.c_str());

    if (esp_now_send(macArray, (uint8_t*)&espNowMessage, sizeof(struct_message)) != ESP_OK) {
        // Serial.println("Failed to send data");
        return false;
    } else {
        // Serial.println("Data sent successfully");
        return true;
    }
}

bool Wireless::sendEspNowJson(uint8_t mac[6], const JsonDocument& jsonCmdInput) {
    serializeJson(jsonCmdInput, espNowMessage.message, sizeof(espNowMessage.message));

    // if (esp_now_send(mac, (uint8_t*)outputString, strlen(outputString)) != ESP_OK) {
    if (esp_now_send(mac, (uint8_t*)&espNowMessage, sizeof(struct_message)) != ESP_OK) {
        // Serial.println("Failed to send data");
        return false;
    } else {
        // Serial.println("Data sent successfully");
        return true;
    }
}

void Wireless::setJsonCommandCallback(JsonCommandCallback callback) {
    jsonCommandCallback = callback;
}

void Wireless::addMacToPeerString(String macInput) {
    printKnownMacs();
    if (macInput.length() != 17) {
        Serial.println("invalid MAC address format.");
        return;
    }
    uint8_t macArray[6];
    macStringToByteArray(macInput, macArray);

    // esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, macArray, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
    MacAddress newMac;
    memcpy(newMac.data(), macArray, 6);
    knownMacs.push_back(newMac);
    Serial.print("Peer added successfully: ");
    Serial.println(macInput);
    printKnownMacs();
}

void Wireless::addMacToPeer(uint8_t mac[6]) {
    printKnownMacs();
    memcpy(peerInfo.peer_addr, mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    } else {
        Serial.print("Peer added successfully: ");
        Serial.println(macToString(mac));

        MacAddress newMac;
        memcpy(newMac.data(), mac, 6);
        knownMacs.push_back(newMac);
    }
    printKnownMacs();
}