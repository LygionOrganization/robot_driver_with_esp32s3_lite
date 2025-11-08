#include "espasyncbutton.hpp"

#define BUTTON_UP    GPIO_NUM_10
#define BUTTON_DOWN  GPIO_NUM_11
#define BUTTON_LEFT  GPIO_NUM_12
#define BUTTON_RIGHT GPIO_NUM_9
#define BUTTON_OK    GPIO_NUM_2

using ESPButton::event_t;

GPIOButton<ESPEventPolicy> bUp(BUTTON_UP, LOW);
GPIOButton<ESPEventPolicy> bDown(BUTTON_DOWN, LOW);
GPIOButton<ESPEventPolicy> bLeft(BUTTON_LEFT, LOW);
GPIOButton<ESPEventPolicy> bRight(BUTTON_RIGHT, LOW);
GPIOButton<ESPEventPolicy> bOK(BUTTON_OK, LOW);

ButtonCallbackMenu menu;
bool settingLeader = false;

void runMission(String missionName, int intervalTime, int loopTimes);
void buttonEventHandler(event_t e, const EventMsg* m){
#ifdef USE_ROBOTIC_ARM
    switch(e){
        case event_t::longPress :
            if (m->gpio == BUTTON_UP){
                buttonBuzzer();
                espnowMode == 0;
                jointsCtrl.allLedCtrl(40, 255, 32, 0);
                jointsCtrl.torqueLock(254, 0);
                jointsCtrl.torqueLockMode = false;
                settingLeader = true;
                screenCtrl.clearDisplay();
                screenCtrl.changeSingleLine(1, "TorqueLock <OFF>", 0);
                if (jointsCtrl.espnowLeader) {
                    jointsCtrl.allLedCtrl(40, 0, 32, 255);
                    screenCtrl.changeSingleLine(2, "Leader BRD <ON>", 0);
                } else {
                    jointsCtrl.allLedCtrl(40, 0, 0, 0);
                    screenCtrl.changeSingleLine(2, "Leader BRD <OFF>", 0);
                }
                screenCtrl.changeSingleLine(3, "longPress-L: ON", 0);
                screenCtrl.changeSingleLine(4, "longPress-R: OFF", 1);
            } else if (m->gpio == BUTTON_DOWN){
                buttonBuzzer();
                settingLeader = false;
                jointsCtrl.espnowLeader = false;
                jointsCtrl.allLedCtrl(40, 0, 0, 0);
                jointsCtrl.torqueLock(254, 1);
                if (jointsCtrl.fineTuningMode) {
                    jointsCtrl.setCurrentSCPosMiddle();
                    jsonFeedback.clear();
                    jsonFeedback["T"] = CMD_SET_JOINTS_ZERO;
                    jsonFeedback["pos"][0] = jointsCtrl.jointsZeroPos[0];
                    jsonFeedback["pos"][1] = jointsCtrl.jointsZeroPos[1];
                    jsonFeedback["pos"][2] = jointsCtrl.jointsZeroPos[2];
                    jsonFeedback["pos"][3] = jointsCtrl.jointsZeroPos[3];
                    serializeJson(jsonFeedback, outputString);
                    filesCtrl.appendStep("boot", outputString);
                    msg(outputString);
                    jointsCtrl.fineTuningMode = false;
                } else {
                    String line_1 = "STA:" + wireless.getSTAIP();
                    String line_2 = "MAC:" + wireless.getMac();
                    String line_3;
                    if (espnowMode == 0) {
                        jointsCtrl.allLedCtrl(40, 0, 0, 0);
                        line_3 = "ESP-NOW <OFF>";
                    } else if (espnowMode == 1) {
                        jointsCtrl.allLedCtrl(40, 64, 0, 255);
                        line_3 = "ESP-NOW <ON> KnownMAC";
                    } else if (espnowMode == 2) {
                        jointsCtrl.allLedCtrl(40, 64, 0, 255);
                        line_3 = "ESP-NOW <ON> BRD MAC";
                    }
                    String line_4;
                    if (jointsCtrl.espnowLeader) {
                        jointsCtrl.allLedCtrl(40, 0, 32, 255);
                        line_4 = "BRD Leader <ON>";
                    } else {
                        jointsCtrl.allLedCtrl(40, 0, 0, 0);
                        line_4 = "BRD Leader <OFF>";
                    }
                    screenCtrl.changeSingleLine(1, line_1, 0);
                    screenCtrl.changeSingleLine(2, line_2, 0);
                    screenCtrl.changeSingleLine(3, line_3, 0);
                    screenCtrl.changeSingleLine(4, line_4, 1);
                }
            } else if (m->gpio == BUTTON_LEFT) {
                buttonBuzzer();
                if (settingLeader) {
                    jointsCtrl.espnowLeader = true;
                    screenCtrl.changeSingleLine(1, "TorqueLock <OFF>", 0);
                    if (jointsCtrl.espnowLeader) {
                        jointsCtrl.allLedCtrl(40, 0, 32, 255);
                        screenCtrl.changeSingleLine(2, "Leader BRD <ON>", 0);
                    } else {
                        jointsCtrl.allLedCtrl(40, 0, 0, 0);
                        screenCtrl.changeSingleLine(2, "Leader BRD <OFF>", 0);
                    }
                    screenCtrl.changeSingleLine(3, "longPress-L: ON", 0);
                    screenCtrl.changeSingleLine(4, "longPress-R: OFF", 1);
                } else {
                    if (espnowMode >= 2) {
                        espnowMode = 1;
                        wireless.setEspNowMode(espnowMode);
                    } else {
                        espnowMode++;
                        wireless.setEspNowMode(espnowMode);
                    }
                    String line_1 = "STA:" + wireless.getSTAIP();
                    String line_2 = "MAC:" + wireless.getMac();
                    String line_3;
                    if (espnowMode == 0) {
                        line_3 = "ESP-NOW <OFF>";
                        jointsCtrl.allLedCtrl(40, 0, 0, 0);
                    } else if (espnowMode == 1) {
                        jointsCtrl.allLedCtrl(40, 64, 0, 255);
                        line_3 = "ESP-NOW <ON> KnownMAC";
                    } else if (espnowMode == 2) {
                        jointsCtrl.allLedCtrl(40, 64, 0, 255);
                        line_3 = "ESP-NOW <ON> BRD MAC";
                    }
                    String line_4;
                    if (jointsCtrl.espnowLeader) {
                        line_4 = "BRD Leader <ON>";
                    } else {
                        line_4 = "BRD Leader <OFF>";
                    }
                    screenCtrl.changeSingleLine(1, line_1, 0);
                    screenCtrl.changeSingleLine(2, line_2, 0);
                    screenCtrl.changeSingleLine(3, line_3, 0);
                    screenCtrl.changeSingleLine(4, line_4, 1);
                }
            } else if (m->gpio == BUTTON_RIGHT) {
                buttonBuzzer();
                if (settingLeader) {
                    jointsCtrl.espnowLeader = false;
                    screenCtrl.changeSingleLine(1, "TorqueLock <OFF>", 0);
                    if (jointsCtrl.espnowLeader) {
                        jointsCtrl.allLedCtrl(40, 0, 32, 255);
                        screenCtrl.changeSingleLine(2, "Leader BRD <ON>", 0);
                    } else {
                        jointsCtrl.allLedCtrl(40, 0, 0, 0);
                        screenCtrl.changeSingleLine(2, "Leader BRD <OFF>", 0);
                    }
                    screenCtrl.changeSingleLine(3, "longPress-L: ON", 0);
                    screenCtrl.changeSingleLine(4, "longPress-R: OFF", 1);
                } else {
                    espnowMode = 0;
                    wireless.setEspNowMode(espnowMode);
                    jointsCtrl.allLedCtrl(40, 0, 0, 0);

                    String line_1 = "STA:" + wireless.getSTAIP();
                    String line_2 = "MAC:" + wireless.getMac();
                    String line_3;
                    if (espnowMode == 0) {
                        line_3 = "ESP-NOW <OFF>";
                    } else if (espnowMode == 1) {
                        jointsCtrl.allLedCtrl(40, 64, 0, 255);
                        line_3 = "ESP-NOW <ON> KnownMAC";
                    } else if (espnowMode == 2) {
                        jointsCtrl.allLedCtrl(40, 64, 0, 255);
                        line_3 = "ESP-NOW <ON> BRD MAC";
                    }
                    String line_4;
                    if (jointsCtrl.espnowLeader) {
                        line_4 = "BRD Leader <ON>";
                    } else {
                        line_4 = "BRD Leader <OFF>";
                    }
                    screenCtrl.changeSingleLine(1, line_1, 0);
                    screenCtrl.changeSingleLine(2, line_2, 0);
                    screenCtrl.changeSingleLine(3, line_3, 0);
                    screenCtrl.changeSingleLine(4, line_4, 1);
                }
            }

    }
#else
    switch(e){
        // Use click event to move cursor Up and Down (just print message)
        case event_t::click :
            // we catch all 'click' events and check which gpio triggered it
            if (m->gpio == BUTTON_UP){
                buttonBuzzer();
                runMission("up", 0, 1);
            } else if (m->gpio == BUTTON_DOWN){
                buttonBuzzer();
                runMission("down", 0, 1);
            } else if (m->gpio == BUTTON_LEFT){
                buttonBuzzer();
                runMission("left", 0, 1);
            } else if (m->gpio == BUTTON_RIGHT){
                buttonBuzzer();
                runMission("right", 0, 1);
            } else if (m->gpio == BUTTON_OK){
                buttonBuzzer();
            }
            break;
    }
#endif
}



void menu_init_RD() {
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);

    if (err == ESP_ERR_WIFI_NOT_INIT) {
        msg("WiFi not initialized. Creating default event loop...");
        ESP_ERROR_CHECK(esp_event_loop_create_default());
    } else {
        msg("WiFi already initialized. No need to create default event loop.");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(EBTN_EVENTS,
        ESP_EVENT_ANY_ID,
        // this lambda will simply translate loop events into btn_callback_t callback function
        [](void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
            menu.handleEvent(ESPButton::int2event_t(id), reinterpret_cast<EventMsg*>(event_data));
        }, 
        NULL, NULL)
    );

    bUp.enableEvent(event_t::click);
    bDown.enableEvent(event_t::click);
    bLeft.enableEvent(event_t::click);
    bRight.enableEvent(event_t::click);
    bOK.enableEvent(event_t::click);

    bUp.enableEvent(event_t::longPress);
    bDown.enableEvent(event_t::longPress);
    bLeft.enableEvent(event_t::longPress);
    bRight.enableEvent(event_t::longPress);

    menu.assign(BUTTON_UP, 0, buttonEventHandler);
    menu.assign(BUTTON_DOWN, 0, buttonEventHandler);
    menu.assign(BUTTON_LEFT, 0, buttonEventHandler);
    menu.assign(BUTTON_RIGHT, 0, buttonEventHandler);
    menu.assign(BUTTON_OK, 0, buttonEventHandler);

    bUp.enable();
    bDown.enable();
    bLeft.enable();
    bRight.enable();
    bOK.enable();
}