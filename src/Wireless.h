#ifndef WIRELESS_H
#define WIRELESS_H

#include <Arduino.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include "Web_page.h"

typedef std::array<uint8_t, 6> MacAddress;

typedef void (*JsonCommandCallback)(const char* jsonString);
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 0 -> no esp-now receive[default]
// 1 -> esp-now on, recv cmd from known MAC addr
// 2 -> esp-now on, recv broadcast cmd
static int espnowMode = 0;

class Wireless{
    public:
        bool fakeMac = false;
        bool setAP(String ssid, String password, int wifiChannel);
        bool setSTA(String ssid, String password);
        bool setWifiMode(int mode, String ap_ssid, String ap_password, int wifiChannel, String sta_ssid, String sta_password);
        int getRSSI_AP();
        int getRSSI_STA();
        String getAPIP();
        String getSTAIP();

        void printKnownMacs();
        bool isKnownMac(const uint8_t *mac);
        void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
        static void IRAM_ATTR staticRecvCallback(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
        
        void espnowInit(bool longRange);
        bool setEspNowMode(int mode);
        int getEspNowMode();
        void macStringToByteArray(const String& macString, uint8_t* byteArray);
        String macToString(uint8_t mac[6]);
        String getMac();
        bool sendEspNow(String macInput, String data);
        bool sendEspNowJson(uint8_t mac[6], const JsonDocument& jsonCmdInput);
        void setJsonCommandCallback(JsonCommandCallback callback);
        void addMacToPeerString(String macInput);
        void addMacToPeer(uint8_t mac[6]);
    private:
        std::vector<MacAddress> knownMacs;
};

#endif