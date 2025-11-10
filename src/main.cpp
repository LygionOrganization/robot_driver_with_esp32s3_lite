// for PlatformIO. This is the main file of the project.
// This file is the entry point of the program.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <nvs_flash.h>
#include "sbus.h"
#include "USB.h"
#include "USBCDC.h"
#include "tusb.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
QueueHandle_t cmdQueue;
#include "Config.h"
#include "jointsCtrl.h"
#include "FilesCtrl.h"
#include "ScreenCtrl.h"
#include "Wireless.h"

JsonDocument jsonCmdReceive;
JsonDocument jsonFeedback;
/*
Declare USBSerial as an instance of USBCDC
Serial is used for USB CDC communication
Serial0 is used for UART communication
*/
// USBCDC USBSerial;
DeserializationError err;
String outputString;
uint8_t* newCmdChar;
JointsCtrl jointsCtrl;
FilesCtrl filesCtrl;
ScreenCtrl screenCtrl;
Wireless wireless;

#define CMD_MAX_LEN 256
#define CMD_QUEUE_LEN 50

bool breakloop = false;
unsigned long tuneStartTime;
int* jointFeedback;

int jointsZeroPos[JOINTS_NUM] = {511, 511, 511, 511};
int jointsCurrentPos[JOINTS_NUM];
double jointFBRads[JOINTS_NUM];
int jointFBTorques[JOINTS_NUM];
double jointsGoalBuffer[JOINTS_NUM];
double xyzgIKFeedback[JOINTS_NUM + 1];
double rbzgIKFeedback[JOINTS_NUM + 1];
double xyzgIK[JOINTS_NUM + 1];
double rbzgIK[JOINTS_NUM + 1];

void jsonCmdReceiveHandler(const JsonDocument& jsonCmdInput);
void addJsonCmd2Queue(const char* jsonString);
void runMission(String missionName, int intervalTime, int loopTimes);

unsigned long startTime;
unsigned long endTime;

#ifdef UART0_AS_SBUS
bfs::SbusRx sbus(&Serial0, 44, 43, true);
bfs::SbusData sbusData;
#endif

void msg(String msgStr, bool newLine = true) {
#ifndef UART0_AS_SBUS
  if (uartMsgFlag) {
    if (newLine) {
      Serial0.println(msgStr);
    } else {
      Serial0.print(msgStr);
    }
  }
#endif
  if (usbMsgFlag) {
    if (newLine) {
      Serial.println(msgStr);
    } else {
      Serial.print(msgStr);
    }
  }
}

void getMsgStatus() {
  jsonFeedback.clear();
  jsonFeedback["T"] = CMD_SET_MSG_OUTPUT;
  jsonFeedback["echo"] = echoMsgFlag;
  jsonFeedback["uart"] = uartMsgFlag;
  jsonFeedback["usb"]  = usbMsgFlag;
  serializeJson(jsonFeedback, outputString);
  msg(outputString);
}

void buttonBuzzer() {
  for (int f = 1200; f <= 2000; f += 200) {
    tone(BUZZER_PIN, f);
    delay(5);
  }
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, HIGH);
}

#include "buttonUI.h"

// websocket & http
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#include "Can.h"

void handleWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                   AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    // feedback after connection
    jsonFeedback.clear();
    jsonFeedback["type"] = "hello";
    jsonFeedback["id"]   = client->id();
    jsonFeedback["msg"]  = "connected";
    serializeJson(jsonFeedback, outputString);
    client->text(outputString);
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      // recv JSON
      char cmdBuffer[CMD_MAX_LEN];
      memcpy(cmdBuffer, data, len);
      cmdBuffer[len] = '\0';
      
      if (xQueueSend(cmdQueue, cmdBuffer, 0) != pdPASS) {
        msg("Queue full, dropping command");
      }

      if (memmem(data, len, "\"T\":0", 5) != nullptr) {
        char tmp[CMD_MAX_LEN];
        while (xQueueReceive(cmdQueue, tmp, 0) == pdPASS) {}
        breakloop = true;
        msg("breakloop");
      }
    }
  }
}

// auto push telemetry
unsigned int baudrate;
String sta_ip;
String ap_ip;
String mac_addr;
uint32_t lastTick = 0;
void pushTelemetry() {
  if (millis() - lastTick >= FB_INTERVAL_MS) {
    lastTick = millis();
    if (ws.count() > 0) {
      jsonFeedback.clear();
      jsonFeedback["T"]   = 50;
      jsonFeedback["baud"]   = baudrate;
      jsonFeedback["sta"]   = wireless.getSTAIP();
      jsonFeedback["ap"]   = ap_ip;
      jsonFeedback["mac"]   = mac_addr;

      if (WiFi.status() == WL_CONNECTED) {
        jsonFeedback["ssid"] = WiFi.SSID();
      } else {
        jsonFeedback["ssid"] = "Disconnected";
      }
      jsonFeedback["uptime"] = (uint32_t)(millis()/1000);
      serializeJson(jsonFeedback, outputString);
      ws.textAll(outputString);
    }
  }
}

void setupHttpRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  // CORS
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");

  // device info：GET /api/info
  server.on("/api/info", HTTP_GET, [](AsyncWebServerRequest *req){
    // StaticJsonDocument<256> doc;
    jsonFeedback.clear();
    
    jsonFeedback["chip"]   = "ESP32-S3";
    jsonFeedback["sdk"]    = ESP.getSdkVersion();
    jsonFeedback["fw_ver"] = "demo-1.0";
    uint8_t m[6]; WiFi.macAddress(m);
    jsonFeedback["mac"]    = wireless.macToString(m);
    serializeJson(jsonFeedback, outputString);
    req->send(200, "application/json", outputString);
  });

  // cmd：POST /api/cmd  ->
  server.on("/api/cmd", HTTP_POST, [](AsyncWebServerRequest *req){},
    NULL,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total){
      // recv JSON
      char cmdBuffer[CMD_MAX_LEN];
      memcpy(cmdBuffer, data, len);
      cmdBuffer[len] = '\0';
      
      if (xQueueSend(cmdQueue, cmdBuffer, 0) != pdPASS) {
        msg("Queue full, dropping command");
      }

      if (memmem(data, len, "\"T\":0", 5) != nullptr) {
        char tmp[CMD_MAX_LEN];
        while (xQueueReceive(cmdQueue, tmp, 0) == pdPASS) {}
        breakloop = true;
        msg("breakloop");
      }
      req->send(200, "application/json", "recv");
    }
  );

  // WebSocket
  ws.onEvent(handleWsEvent);
  server.addHandler(&ws);
}

void runMission(String missionName, int intervalTime, int loopTimes);
void runMissionTask(void *parameter) {
    runMission("boot_user", 0, -1);
    vTaskDelete(NULL);
}

/**
 * @brief Setup function to initialize the device and its peripherals.
 * 
 * - Waits for the internal capacitors of the joints to charge.
 * - Initializes communication interfaces (S.BUS or Serial0).
 * - Configures I2C communication with specified pins and clock speed.
 * - Sets up the buzzer and triggers a button buzzer action.
 * - Initializes USB CDC communication for the ESP32-S3.
 * - Configures and initializes joint control with specific parameters.
 * - Optionally initializes the screen controller for display (commented out).
 * - If file system usage is enabled, initializes the file system and checks/creates a boot mission.
 * - If ESP-NOW is enabled, initializes wireless communication and sets up a JSON command callback.
 */
void setup() {
  /*
  Waits for the internal capacitors of the joints to charge.
  */
  delay(1000);

#ifdef UART0_AS_SBUS
    sbus.Begin();
    msg("S.BUS mode initialized.");
#else
    Serial0.begin(ESP32S3_BAUD_RATE);
    msg("Serial0 initialized for normal communication.");
    while (!Serial0) {}
#endif
  msg("Device starting...");

  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);

  cmdQueue = xQueueCreate(CMD_QUEUE_LEN, CMD_MAX_LEN);
  if (cmdQueue == NULL) {
    msg("Failed to create command queue!");
  }

  // buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  buttonBuzzer();

  // CAN-2.0
  CanStart();

  // fake args, it will be ignored by the USB stack, default baudrate is 12Mbps
  USBSerial.begin(ESP32S3_BAUD_RATE);
  USB.begin();
  msg("ESP32-S3 USB CDC DONE!");

  // init serial for bus devs
  jointsCtrl.init(BUS_SERVO_BAUD_RATE);

#ifdef USE_UI_CTRL
  screenCtrl.init();
  screenCtrl.displayText("LYgion", 0, 0, 2);
  screenCtrl.displayText("Robotics", 0, 16, 2);
  msg("ScreenCtrl initialized.");
#else
  msg("ScreenCtrl NOT initialized.");
#endif

#ifdef USE_FILE_SYSTEM
  filesCtrl.init();
  if(!filesCtrl.checkMission("boot")) {
    filesCtrl.createMission("boot", "this is the boot mission.");
    filesCtrl.appendStep("boot", "{\"T\":400,\"mode\":1,\"ap_ssid\":\"Robot\",\"ap_password\":\"12345678\",\"channel\":1,\"sta_ssid\":\"\",\"sta_password\":\"\"}");
    msg("creat New mission: boot");
  } else {
    msg("boot mission already exists.");
    if (filesCtrl.checkStepByType("boot", CMD_SET_WIFI_MODE)) {
      msg("Already set wifi mode.");
    } else {
      filesCtrl.appendStep("boot", "{\"T\":400,\"mode\":1,\"ap_ssid\":\"Robot\",\"ap_password\":\"12345678\",\"channel\":1,\"sta_ssid\":\"\",\"sta_password\":\"\"}");
      msg("Haven't set wifi mode yet. Appending to boot mission.");
    }
  }
  runMission("boot", 0, 1);
  delay(200);
  msg("File system initialized.");
#else
  msg("File system NOT initialized.");
#endif

#ifdef USE_ESP_NOW
  wireless.espnowInit(false);
  // wireless.espnowInit(true); // longRange Mode
  wireless.setJsonCommandCallback(addJsonCmd2Queue);
  msg("ESP-NOW initialized.");
#else
  msg("ESP-NOW NOT initialized.");
#endif

#ifndef USE_HUB_MOTORS
  msg("Hub motors NOT initialized.");
#endif

  // buttonUI
  /*
  Assign button events to menu actions
  must be after wifi function
  */
  menu_init_RD();

#ifdef USE_ROBOTIC_ARM
  jointsCtrl.setJointType(JOINT_TYPE_SC);
  jointsCtrl.setEncoderStepRange(1024, 220);
  // jointsCtrl.setJointsZeroPosArray(jointsZeroPos);
  jointsCtrl.allLedCtrl(40, 160, 0, 255);
  wireless.addMacToPeer(broadcastAddress);
  if (!filesCtrl.checkStepByType("boot", CMD_SET_JOINTS_ZERO)) {
    msg("No CMD_SET_JOINTS_ZERO found, torqueLock <OFF>");
    jointsCtrl.setJointsZeroPosArray(jointsZeroPos);
    jointsCtrl.torqueLockMode = false;
    jointsCtrl.fineTuningMode = true;
    jointsCtrl.allLedCtrl(40, 255, 128, 0);

    double maxSpeedBuffer = jointsCtrl.getMaxJointsSpeed();
    jointsCtrl.setMaxJointsSpeed(50);
    double anglesBuffer[4] = {0, 0, 0, -90};
    jointsCtrl.linkArmSCJointsCtrlAngle(anglesBuffer);
    delay(3500);
    jointsCtrl.allLedCtrl(40, 255, 32, 0);
    jointsCtrl.setMaxJointsSpeed(maxSpeedBuffer);

    jointsCtrl.torqueLock(254, 0);
    screenCtrl.changeSingleLine(1, "-----o---o     Manual", 0);
    screenCtrl.changeSingleLine(2, "     |   | Joint Pose", 0);
    screenCtrl.changeSingleLine(3, "     oo--o Status<OK>", 0);
    screenCtrl.changeSingleLine(4, "then longPress <Down>", 1);
  } else {
    double maxSpeedBuffer = jointsCtrl.getMaxJointsSpeed();
    jointsCtrl.setMaxJointsSpeed(0.1);
    jointsCtrl.linkArmFPVIK(LINK_AB + LINK_BF_1 + LINK_EF/2, 
                            0, 
                            LINK_BF_2,
                            GRIPPER_OPEN);
    msg("JointsCtrl initialized.");
    jointsCtrl.setMaxJointsSpeed(maxSpeedBuffer);

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

    jointsCtrl.allLedCtrl(40, 0, 0, 0);
  }
#else
  screenCtrl.clearDisplay();
  String line_1 = filesCtrl.getValueByMissionNameAndKey("boot", CMD_SET_WIFI_MODE, "ap_ssid");
  String line_2 = "AP:" + wireless.getAPIP();
  String line_3 = "STA:" + wireless.getSTAIP();
  String line_4 = "MAC:" + wireless.getMac();
  screenCtrl.changeSingleLine(1, line_1, 0);
  screenCtrl.changeSingleLine(2, line_2, 0);
  screenCtrl.changeSingleLine(3, line_3, 0);
  screenCtrl.changeSingleLine(4, line_4, 1);
#endif

  setupHttpRoutes();
  server.begin();

  baudrate = jointsCtrl.baudrate;
  sta_ip = wireless.getSTAIP();
  ap_ip = wireless.getAPIP();
  mac_addr = wireless.getMac();
  
  // runMission("boot_user", 0, -1);
  xTaskCreatePinnedToCore(
      runMissionTask,
      "MissionTask", 
      8192,            
      NULL,            
      1,               
      NULL,            
      1                
  );

#ifdef CAN_BUS_MACHINE
  screenCtrl.init();
  screenCtrl.displayText("CAN-TEST", 0, 0, 4);
#endif

#ifdef TTL_ADAPTER_MACHINE
  screenCtrl.init();
  screenCtrl.displayText("TTLA IO", 0, 0, 4);
#endif
}

void delayInterruptible(int ms) {
    int elapsed = 0;
    int step = 10;
    while (elapsed < ms) {
        if (breakloop) {
            // breakloop = false;
            return;
        }
        int delayTime = (ms - elapsed > step) ? step : (ms - elapsed);
        delay(delayTime);
        elapsed += delayTime;
    }
}

void breakLoop() {
  breakloop = true;
  char tmp[CMD_MAX_LEN];
  while (xQueueReceive(cmdQueue, tmp, 0) == pdPASS) {}
}

bool runStep(String missionName, int step) {
  outputString = filesCtrl.readStep(missionName, step);
  err = deserializeJson(jsonCmdReceive, outputString);
  if (err == DeserializationError::Ok) {
    jsonCmdReceiveHandler(jsonCmdReceive);
    return true;
  } else {
    msg("JSON parsing error (this is a normal output when booting or running Mission): ");
    msg(err.c_str());
    return false;
  }
}

void runMission(String missionName, int intervalTime, int loopTimes) {
  if (!filesCtrl.checkMission(missionName)) {
    return;
  }
  intervalTime = intervalTime - timeOffset;
  if (intervalTime < 0) {intervalTime = 0;}
  int j = 1;
  while (true) {
    msg("Running loop: ", false);
    msg(String(j));
    int i = 1;
    while (true) {
      if (breakloop) {
        breakloop = false;
        msg("breakloop");
        return;
      }
      if (runStep(missionName, i)) {
        msg("Step: ", false);
        msg(String(i));
        i++;
        delay(intervalTime);
      } else {
        msg("Mission Completed.");
        break;
      }
    }
    j++;
    if (j > loopTimes && loopTimes != -1) {
      return;
    }
  }
}

void addJsonCmd2Queue(const char* jsonString) {
  if (xQueueSend(cmdQueue, jsonString, 0) != pdPASS) {
    msg("Queue full, dropping command");
  }
}

void jsonCmdReceiveHandler(const JsonDocument& jsonCmdInput) {
  int cmdType;
  cmdType = jsonCmdInput["T"].as<int>();
  switch(cmdType){
  // for web ui ctrl
  case CMD_BREAK_LOOP:
                        breakLoop();
                        break;
  case CMD_WEB_SET_JOINTS_BAUD:
                        jointsCtrl.setBaudRate(jsonCmdInput["baud"]);
                        baudrate = jointsCtrl.baudrate;
                        break;
  
  case CMD_STSM_CTRL:
                        jointsCtrl.stepsCtrlSMST(jsonCmdInput["id"],
                                                 jsonCmdInput["pos"],
                                                 jsonCmdInput["spd"],
                                                 jsonCmdInput["acc"],
                                                 true);
                        break;
  case CMD_STSM_SET_MIDDLE:
                        jointsCtrl.setMiddleSTSM(jsonCmdInput["id"]);
                        break;
  case CMD_STSM_CHANGE_ID:
                        jointsCtrl.changeIDSTSM(jsonCmdInput["old_id"], 
                                                jsonCmdInput["new_id"]);
                        break;
  case CMD_STSM_TORQUE_LOCK:
                        jointsCtrl.torqueLockSTSM(jsonCmdInput["id"],
                                                  jsonCmdInput["state"]);
                        break;
  case CMD_STSM_FEEDBACK:
                        jointFeedback = jointsCtrl.feedbackSTSM(jsonCmdInput["id"]);
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_STSM_FEEDBACK;
                        if (jointFeedback[0] == -1) {
                          jsonFeedback["ps"] = -1;
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        } else {
                          jsonFeedback["ps"] = jointFeedback[0];
                          jsonFeedback["pos"] = jointFeedback[1];
                          jsonFeedback["spd"] = jointFeedback[2];
                          jsonFeedback["load"] = jointFeedback[3];
                          jsonFeedback["vol"] = jointFeedback[4];
                          jsonFeedback["temp"] = jointFeedback[5];
                          jsonFeedback["mov"] = jointFeedback[6];
                          jsonFeedback["curt"] = jointFeedback[7];
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;

  case CMD_HL_CTRL:
                        jointsCtrl.stepsCtrlHL(jsonCmdInput["id"],
                                               jsonCmdInput["pos"],
                                               jsonCmdInput["spd"],
                                               jsonCmdInput["acc"],
                                               jsonCmdInput["cl"],
                                               true);
                        break;
  case CMD_HL_SET_MIDDLE:
                        jointsCtrl.setMiddleHL(jsonCmdInput["id"]);
                        break;
  case CMD_HL_CHANGE_ID:
                        jointsCtrl.changeIDHL(jsonCmdInput["old_id"],
                                              jsonCmdInput["new_id"]);
                        break;
  case CMD_HL_TORQUE_LOCK:
                        jointsCtrl.torqueLockHL(jsonCmdInput["id"],
                                                jsonCmdInput["state"]);
                        break;
  case CMD_HL_FEEDBACK:
                        jointFeedback = jointsCtrl.feedbackHL(jsonCmdInput["id"]);
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_HL_FEEDBACK;
                        if (jointFeedback[0] == -1) {
                          jsonFeedback["ps"] = -1;
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        } else {
                          jsonFeedback["ps"] = jointFeedback[0];
                          jsonFeedback["pos"] = jointFeedback[1];
                          jsonFeedback["spd"] = jointFeedback[2];
                          jsonFeedback["load"] = jointFeedback[3];
                          jsonFeedback["vol"] = jointFeedback[4];
                          jsonFeedback["temp"] = jointFeedback[5];
                          jsonFeedback["mov"] = jointFeedback[6];
                          jsonFeedback["curt"] = jointFeedback[7];
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;

  case CMD_SC_CTRL:
                        jointsCtrl.stepsCtrlSC(jsonCmdInput["id"],
                                               jsonCmdInput["pos"],
                                               jsonCmdInput["time"],
                                               jsonCmdInput["spd"],
                                               true);
                        break;
  case CMD_SC_CHANGE_ID:
                        jointsCtrl.changeIDSC(jsonCmdInput["old_id"],
                                              jsonCmdInput["new_id"]);
                        break;
  case CMD_SC_TORQUE_LOCK:
                        jointsCtrl.torqueLockSC(jsonCmdInput["id"],
                                                jsonCmdInput["state"]);
                        break;
  case CMD_SC_FEEDBACK:
                        jointFeedback = jointsCtrl.feedbackSC(jsonCmdInput["id"]);
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_SC_FEEDBACK;
                        if (jointFeedback[0] == -1) {
                          jsonFeedback["ps"] = -1;
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        } else {
                          jsonFeedback["ps"] = jointFeedback[0];
                          jsonFeedback["pos"] = jointFeedback[1];
                          jsonFeedback["spd"] = jointFeedback[2];
                          jsonFeedback["load"] = jointFeedback[3];
                          jsonFeedback["vol"] = jointFeedback[4];
                          jsonFeedback["temp"] = jointFeedback[5];
                          jsonFeedback["mov"] = jointFeedback[6];
                          jsonFeedback["curt"] = jointFeedback[7];
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;
  
  case CMD_DELAY: 
                        delayInterruptible(jsonCmdInput["delay"]);
                        break;



  case CMD_SET_JOINTS_BAUD:
                        jointsCtrl.setBaudRate(jsonCmdInput["baud"]);
                        break;
  case CMD_SET_JOINTS_TYPE:
                        if (jointsCtrl.setJointType(jsonCmdInput["type"])) {
                          msg("Joint type set successfully.");
                        } else {
                          msg("Failed to set joint type.");
                        }
                        break;
  case CMD_SET_ENCODER:
                        if (jointsCtrl.setEncoderStepRange(jsonCmdInput["steps"], jsonCmdInput["angle"])) {
                          msg("Encoder step range set successfully.");
                        } else {
                          msg("Failed to set encoder step range.");
                        }
                        break;
  case CMD_SINGLE_FEEDBACK:
                        jointFeedback = jointsCtrl.singleFeedBack(jsonCmdInput["id"]);
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_SINGLE_FEEDBACK;
                        if (jointFeedback[0] == -1) {
                          jsonFeedback["ps"] = -1;
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        } else {
                          jsonFeedback["ps"] = jointFeedback[0];
                          jsonFeedback["pos"] = jointFeedback[1];
                          jsonFeedback["spd"] = jointFeedback[2];
                          jsonFeedback["load"] = jointFeedback[3];
                          jsonFeedback["vol"] = jointFeedback[4];
                          jsonFeedback["temp"] = jointFeedback[5];
                          jsonFeedback["mov"] = jointFeedback[6];
                          jsonFeedback["curt"] = jointFeedback[7];
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;
  case CMD_PING:
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_PING;
                        if (jointsCtrl.ping(jsonCmdInput["id"])) {
                          jsonFeedback["ps"] = 1;
                        } else {
                          jsonFeedback["ps"] = -1;
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_CHANGE_ID:
                        jointsCtrl.changeID(jsonCmdInput["old_id"], 
                                            jsonCmdInput["new_id"]);
                        break;
  case CMD_SET_MIDDLE:
                        if (jointsCtrl.setMiddle(jsonCmdInput["id"])) {
                          msg("Joint middle set successfully.");
                        } else {
                          msg("Failed to set joint middle.");
                        }
                        break;
  case CMD_MOVE_MIDDLE:
                        jointsCtrl.moveMiddle(jsonCmdInput["id"]);
                        break;
  case CMD_TORQUE_LOCK:
                        jointsCtrl.torqueLock(jsonCmdInput["id"], 
                                              jsonCmdInput["state"]);
                        break;
  case CMD_STEPS_CTRL_SC:
                        jointsCtrl.stepsCtrlSC(jsonCmdInput["id"], 
                                               jsonCmdInput["pos"], 
                                               jsonCmdInput["time"], 
                                               jsonCmdInput["spd"], 
                                               jsonCmdInput["move"]);
                        break;
  case CMD_STEPS_CTRL_SMST:
                        // void
                        break;
  case CMD_STEPS_CTRL_HL:
                        // void
                        break;
  case CMD_ANGLE_CTRL_SC:
                        jointsCtrl.angleCtrlSC(jsonCmdInput["id"], 
                                               jsonCmdInput["mid"], 
                                               jsonCmdInput["ang"], 
                                               jsonCmdInput["spd"], 
                                               jsonCmdInput["move"]);
                        break;
  case CMD_RAD_CTRL_SC:
                        jointsCtrl.radCtrlSC(jsonCmdInput["id"],
                                             jsonCmdInput["mid"],
                                             jsonCmdInput["rad"],
                                             jsonCmdInput["spd"],
                                             jsonCmdInput["move"]);
                        break;
  case CMD_MOVE_TRIGGER:
                        jointsCtrl.moveTrigger();
                        break;



  // --- --- --- for applications: LyLinkArm --- --- ---
  case CMD_GET_JOINTS_ZERO:
                        memcpy(jointsZeroPos, jointsCtrl.getJointsZeroPosArray(), sizeof(jointsZeroPos));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_GET_JOINTS_ZERO;
                        for (int i = 0; i < JOINTS_NUM; i++) {  
                            jsonFeedback["pos"][i] = jointsZeroPos[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_SET_JOINTS_ZERO:
                        for (int i = 0; i < JOINTS_NUM; i++) {
                            jointsZeroPos[i] = jsonCmdInput["pos"][i];
                            
                        }
                        jointsCtrl.setJointsZeroPosArray(jointsZeroPos);
                        break;
  case CMD_GET_LINK_ARM_POS:
                        memcpy(jointsCurrentPos, jointsCtrl.getLinkArmPosSC(), sizeof(jointsCurrentPos));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_GET_LINK_ARM_POS;
                        for (int i = 0; i < JOINTS_NUM; i++) {
                            jsonFeedback["pos"][i] = jointsCurrentPos[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_SET_LINK_ARM_ZERO:
                        jointsCtrl.setCurrentSCPosMiddle();
                        memcpy(jointsCurrentPos, jointsCtrl.getLinkArmPosSC(), sizeof(jointsCurrentPos));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = CMD_SET_JOINTS_ZERO;
                        for (int i = 0; i < JOINTS_NUM; i++) {
                            jsonFeedback["pos"][i] = jointsCurrentPos[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_LINK_ARM_SC_JOINTS_CTRL_ANGLE:
                        for (int i = 0; i < JOINTS_NUM; i++) {
                            jointsGoalBuffer[i] = jsonCmdInput["ang"][i];
                        }
                        jointsCtrl.linkArmSCJointsCtrlAngle(jointsGoalBuffer);
                        break;
  case CMD_LINK_ARM_SC_JOINTS_CTRL_RAD:
                        for (int i = 0; i < JOINTS_NUM; i++) {
                            jointsGoalBuffer[i] = jsonCmdInput["rad"][i];
                        }
                        jointsCtrl.linkArmSCJointsCtrlRad(jointsGoalBuffer);
                        break;
  case CMD_XYZG_CTRL:
                        memcpy(xyzgIK,
                        jointsCtrl.linkArmSpaceIK(jsonCmdInput["xyzg"][0],
                                                  jsonCmdInput["xyzg"][1],
                                                  jsonCmdInput["xyzg"][2],
                                                  jsonCmdInput["xyzg"][3]),
                        sizeof(xyzgIK));
                        if (xyzgIK[0] == -1) {
                          jsonFeedback.clear();
                          jsonFeedback["T"] = -CMD_XYZG_CTRL;
                          jsonFeedback["ik"] = xyzgIK[0];
                          for (int i = 0; i < JOINTS_NUM; i++) {
                            jsonFeedback["xyzg"][i] = xyzgIK[i + 1];
                          }
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;
  case CMD_FPV_ABS_CTRL:
                        memcpy(rbzgIKFeedback, 
                               jointsCtrl.linkArmFPVIK(jsonCmdInput["rbzg"][0],
                                                      jsonCmdInput["rbzg"][1],
                                                      jsonCmdInput["rbzg"][2],
                                                      jsonCmdInput["rbzg"][3]), 
                               sizeof(rbzgIKFeedback));
                        if (rbzgIKFeedback[0] == -1) {
                          jsonFeedback.clear();
                          jsonFeedback["T"] = -CMD_FPV_ABS_CTRL;
                          jsonFeedback["ik"] = rbzgIKFeedback[0];
                          for (int i = 0; i < JOINTS_NUM; i++) {
                            jsonFeedback["rbzg"][i] = rbzgIKFeedback[i + 1];
                          }
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;
  case CMD_SMOOTH_XYZG_CTRL:
                        memcpy(xyzgIKFeedback, 
                          jointsCtrl.smoothXYZGCtrl(jsonCmdInput["xyzg"][0],
                                                    jsonCmdInput["xyzg"][1],
                                                    jsonCmdInput["xyzg"][2],
                                                    jsonCmdInput["xyzg"][3],
                                                    jsonCmdInput["spd"]), 
                          sizeof(xyzgIKFeedback));
                        if (xyzgIKFeedback[0] == -1) {
                          jsonFeedback.clear();
                          jsonFeedback["T"] = -CMD_SMOOTH_XYZG_CTRL;
                          jsonFeedback["ik"] = xyzgIKFeedback[0];
                          for (int i = 0; i < JOINTS_NUM; i++) {
                            jsonFeedback["xyzg"][i] = xyzgIKFeedback[i + 1];
                          }
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;
  case CMD_SMOOTH_FPV_ABS_CTRL:
                        memcpy(rbzgIKFeedback, 
                               jointsCtrl.smoothFPVAbsCtrl(jsonCmdInput["rbzg"][0],
                                                           jsonCmdInput["rbzg"][1],
                                                           jsonCmdInput["rbzg"][2],
                                                           jsonCmdInput["rbzg"][3],
                                                           jsonCmdInput["spd"],
                                                           jsonCmdInput["br"]), 
                               sizeof(rbzgIKFeedback));
                        if (rbzgIKFeedback[0] == -1) {
                          jsonFeedback.clear();
                          jsonFeedback["T"] = -CMD_FPV_ABS_CTRL;
                          jsonFeedback["ik"] = rbzgIKFeedback[0];
                          for (int i = 0; i < JOINTS_NUM; i++) {
                            jsonFeedback["rbzg"][i] = rbzgIKFeedback[i + 1];
                          }
                          serializeJson(jsonFeedback, outputString);
                          msg(outputString);
                        }
                        break;
  case CMD_SET_MAX_JOINTS_SPEED:
                        jointsCtrl.setMaxJointsSpeed(jsonCmdInput["spd"]);
                        break;
  case CMD_GET_JOINT_FB_RADS:
                        memcpy(jointFBRads, jointsCtrl.getJointFBRads(), sizeof(jointFBRads));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_GET_JOINT_FB_RADS;
                        for (int i = 0; i < JOINTS_NUM; i++) {
                          jsonFeedback["fb-rads"][i] = jointFBRads[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_GET_JOINT_GOAL_RADS:
                        memcpy(jointsGoalBuffer, jointsCtrl.getJointGoalRads(), sizeof(jointsGoalBuffer));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_GET_JOINT_GOAL_RADS;
                        for (int i = 0; i < JOINTS_NUM; i++) {
                          jsonFeedback["goal-rads"][i] = jointsGoalBuffer[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_GET_XYZG_IK:
                        memcpy(xyzgIK, jointsCtrl.getXYZGIK(), sizeof(xyzgIK));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_GET_XYZG_IK;
                        for (int i = 0; i < JOINTS_NUM + 1; i++) {
                          jsonFeedback["xyzg"][i] = xyzgIK[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_GET_RBZG_IK:
                        memcpy(rbzgIK, jointsCtrl.getRBZGIK(), sizeof(rbzgIK));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = -CMD_GET_RBZG_IK;
                        for (int i = 0; i < JOINTS_NUM + 1; i++) {
                          jsonFeedback["rbzg"][i] = rbzgIK[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        msg(outputString);
                        break;
  case CMD_SET_LINK_ARM_FEEDBACK_FLAG:
                        if (jsonCmdInput["flag"] == 0) {
                          jointsCtrl.setLinkArmFeedbackFlag(false, jsonCmdInput["hz"]);
                        } else if (jsonCmdInput["flag"] == 1) {
                          jointsCtrl.setLinkArmFeedbackFlag(true, jsonCmdInput["hz"]);
                        }
                        break;






  case CMD_HUB_MOTOR_CTRL:
                        jointsCtrl.hubMotorCtrl(jsonCmdInput["A"], 
                                                jsonCmdInput["B"], 
                                                jsonCmdInput["C"], 
                                                jsonCmdInput["D"]);
                        break;





  case CMD_SET_SINGLE_COLOR:
                        jointsCtrl.singleLedCtrl(jsonCmdInput["id"],
                                                 jsonCmdInput["set"][0],
                                                 jsonCmdInput["set"][1],
                                                 jsonCmdInput["set"][2],
                                                 jsonCmdInput["set"][3]);
                        break;
  case CMD_SET_ALL_COLOR:
                        jointsCtrl.allLedCtrl(jsonCmdInput["id"],
                                              jsonCmdInput["set"][0],
                                              jsonCmdInput["set"][1],
                                              jsonCmdInput["set"][2]);
                        break;
  case CMD_DISPLAY_SINGLE:
                        screenCtrl.changeSingleLine(jsonCmdInput["line"], 
                                                    jsonCmdInput["text"], 
                                                    jsonCmdInput["update"]);
                        break;
  case CMD_DISPLAY_UPDATE:
                        screenCtrl.updateFrame();
                        break;
                        
  case CMD_DISPLAY_FRAME:
                        screenCtrl.changeSingleLine(1, jsonCmdInput["l1"], false);
                        screenCtrl.changeSingleLine(2, jsonCmdInput["l2"], false);
                        screenCtrl.changeSingleLine(3, jsonCmdInput["l3"], false);
                        screenCtrl.changeSingleLine(4, jsonCmdInput["l4"], true);
                        break;
  case CMD_DISPLAY_CLEAR:
                        screenCtrl.clearDisplay();
                        break;
  case CMD_BUZZER_CTRL:
                        tone(BUZZER_PIN, jsonCmdInput["freq"]);
                        tuneStartTime = millis();
                        while (millis() - tuneStartTime < jsonCmdInput["duration"]) {
                          if (breakloop) {
                            breakloop = false;
                            noTone(BUZZER_PIN);
                            digitalWrite(BUZZER_PIN, HIGH);
                            break;
                          }
                        }
                        noTone(BUZZER_PIN);
                        digitalWrite(BUZZER_PIN, HIGH);
                        break;
  // case CMD_BUTTONS:
  //                       buttonInteractionCtrl(jsonCmdInput["L"],
  //                                             jsonCmdInput["I0"],
  //                                             jsonCmdInput["I1"],
  //                                             jsonCmdInput["I2"]);
  //                       break;



  case CMD_SCAN_FILES:
                        filesCtrl.scan();
                        break;
  case CMD_CREATE_MISSION:
                        filesCtrl.createMission(jsonCmdInput["name"], 
                                                jsonCmdInput["intro"]);
                        break;
  case CMD_FORMAT_FLASH:
                        filesCtrl.flash();
                        break;
  case CMD_MISSION_CONTENT:
                        filesCtrl.missionContent(jsonCmdInput["name"]);
                        break;
  case CMD_APPEND_SETP_JSON:
                        filesCtrl.appendStep(jsonCmdInput["name"],
                                             jsonCmdInput["json"]);
                        break;
  case CMD_INSERT_SETP_JSON:
                        filesCtrl.insertStep(jsonCmdInput["name"],
                                             jsonCmdInput["step"],
                                             jsonCmdInput["json"]);
                        break;
  case CMD_REPLACE_SETP_JSON:
                        filesCtrl.replaceStep(jsonCmdInput["name"],
                                              jsonCmdInput["step"],
                                              jsonCmdInput["json"]);
                        break;
  case CMD_DELETE_SETP:
                        filesCtrl.deleteStep(jsonCmdInput["name"],
                                             jsonCmdInput["step"]);
                        break;
  case CMD_RUN_STEP:    
                        runStep(jsonCmdInput["name"], jsonCmdInput["step"]);
                        break;
  case CMD_RUN_MISSION: 
                        runMission(jsonCmdInput["name"], 
                                   jsonCmdInput["interval"], 
                                   jsonCmdInput["loop"]);
                        break;
  case CMD_DELETE_MISSION:
                        filesCtrl.deleteMission(jsonCmdInput["name"]);
                        break;
  


  case CMD_SET_WIFI_MODE:
                        if (jsonCmdInput["mode"] == 0) {
                          wireless.setWifiMode(jsonCmdInput["mode"], "", "", 0, "", "");
                          jsonFeedback.clear();
                          jsonFeedback["T"] = CMD_SET_WIFI_MODE;
                          jsonFeedback["mode"] = 0;
                          serializeJson(jsonFeedback, outputString);
                          filesCtrl.checkReplaceStep("boot", outputString);
                        } else if (jsonCmdInput["mode"] == 1) {
                          if (wireless.setWifiMode(jsonCmdInput["mode"], 
                                                   jsonCmdInput["ap_ssid"], 
                                                   jsonCmdInput["ap_password"], 
                                                   jsonCmdInput["channel"], 
                                                   jsonCmdInput["sta_ssid"], 
                                                   jsonCmdInput["sta_password"])) {
                              jsonFeedback.clear();
                              jsonFeedback["T"] = CMD_SET_WIFI_MODE;
                              jsonFeedback["mode"] = 1;
                              jsonFeedback["ap_ssid"] = jsonCmdInput["ap_ssid"];
                              jsonFeedback["ap_password"] = jsonCmdInput["ap_password"];
                              jsonFeedback["channel"] = jsonCmdInput["channel"];
                              jsonFeedback["sta_ssid"] = jsonCmdInput["sta_ssid"];
                              jsonFeedback["sta_password"] = jsonCmdInput["sta_password"];
                              serializeJson(jsonFeedback, outputString);
                              filesCtrl.checkReplaceStep("boot", outputString);
                          } else {
                              jsonFeedback.clear();
                              jsonFeedback["T"] = CMD_SET_WIFI_MODE;
                              jsonFeedback["mode"] = 1;
                              jsonFeedback["ap_ssid"] = jsonCmdInput["ap_ssid"];
                              jsonFeedback["ap_password"] = jsonCmdInput["ap_password"];
                              jsonFeedback["channel"] = jsonCmdInput["channel"];
                              jsonFeedback["sta_ssid"] = "";
                              jsonFeedback["sta_password"] = "";
                              serializeJson(jsonFeedback, outputString);
                              filesCtrl.checkReplaceStep("boot", outputString);
                          }
                        }
                        break;
  case CMD_WIFI_INFO:
                        outputString = filesCtrl.findCmdByType("boot", CMD_SET_WIFI_MODE);
                        msg(outputString);
                        break;
  case CMD_GET_AP_IP:
                        outputString = wireless.getAPIP();
                        msg(outputString);
                        break;
  case CMD_GET_STA_IP:
                        outputString = wireless.getSTAIP();
                        msg(outputString);
                        break;



  case CMD_INIT_ESP_NOW:
                        wireless.espnowInit(jsonCmdInput["longrange"]);
                        break;
  case CMD_SET_ESP_NOW_MODE:
                        wireless.setEspNowMode(jsonCmdInput["mode"]);
                        break;
  case CMD_GET_MAC:
                        outputString = wireless.getMac();
                        msg(outputString);
                        break;
  case CMD_ESP_NOW_SEND:
                        wireless.sendEspNow(jsonCmdInput["mac"], jsonCmdInput["data"]);
                        break;
  case CMD_ADD_MAC:
                        wireless.addMacToPeerString(jsonCmdInput["mac"]);
                        break;
  case CMD_FAKE_MAC:    wireless.fakeMac = jsonCmdInput["fake"];
                        break;
  


  case CMD_CAN_SEND:
                        sendJsonAsCAN(jsonCmdInput);
                        break;



  case CMD_ESP32_REBOOT:
                        ESP.restart();
                        break;
  case CMD_CLEAR_NVS:
                        nvs_flash_erase();
                        delay(1000);
                        nvs_flash_init();
                        break;
  case CMD_RESET:
                        filesCtrl.flash();
                        break;
  case CMD_GET_MSG_OUTPUT:
                        getMsgStatus();
                        break;
  case CMD_SET_MSG_OUTPUT:
                        echoMsgFlag = jsonCmdInput["echo"];
                        uartMsgFlag = jsonCmdInput["uart"];
                        usbMsgFlag  = jsonCmdInput["usb"];
                        break;
  case CMD_SERIAL_FORWARDING:
                        if (jsonCmdInput["sf"] == 1){
                          serialForwarding = true;
                        } else if (jsonCmdInput["sf"] == 0) {
                          serialForwarding = false;
                        }
                        break;
  
  case CMD_TEST:
                        jointsCtrl.setBaudRate(1000000);
                        // jointsCtrl.stepsCtrlSC(254, 200, 0, 0, true);
                        jointsCtrl.stepsCtrlSMST(254, 0, 0, 0, true);
                        msg("test - 1");
                        delay(2000);
                        // jointsCtrl.stepsCtrlSC(254, 800, 0, 0, true);
                        jointsCtrl.stepsCtrlSMST(254, 4000, 0, 0, true);
                        msg("test - 2");
                        delay(2000);
                        // jointsCtrl.stepsCtrlSC(254, 200, 0, 0, true);
                        jointsCtrl.stepsCtrlSMST(254, 0, 0, 0, true);
                        msg("test - 3");
                        delay(2000);
                        // jointsCtrl.stepsCtrlSC(254, 800, 0, 0, true);
                        jointsCtrl.stepsCtrlSMST(254, 4000, 0, 0, true);
                        msg("test - 4");
                        break;
  
  }
}


// USB CDC receive callback
void tud_cdc_rx_cb(uint8_t itf) {
  static String receivedData;
  char buffer[256];
  uint32_t count = tud_cdc_read(buffer, sizeof(buffer));

  for (uint32_t i = 0; i < count; i++) {
    char receivedChar = buffer[i];
    receivedData += receivedChar;

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      // Now we have received the complete JSON string
      char cmdBuffer[CMD_MAX_LEN];
      size_t len = receivedData.length();
      if (len >= CMD_MAX_LEN) len = CMD_MAX_LEN - 1;
      newCmdChar = (uint8_t*)receivedData.c_str();
      memcpy(cmdBuffer, newCmdChar, len);
      cmdBuffer[len] = '\0';

      if (xQueueSend(cmdQueue, cmdBuffer, 0) != pdPASS) {
        msg("Queue full, dropping command");
      }
      
      if (memmem(newCmdChar, len, "\"T\":0", 5) != nullptr) {
        char tmp[CMD_MAX_LEN];
        while (xQueueReceive(cmdQueue, tmp, 0) == pdPASS) {}
        breakloop = true;
        msg("breakloop");
      }
      receivedData = "";
    }
  }
}


void serialCtrl() {
  if (serialForwarding) {
    // Serial0 -> Serial1
    while (Serial0.available()) {
      int data = Serial0.read();
      Serial1.write(data);
    }
    // Serial1 -> Serial0
    while (Serial1.available()) {
      int data = Serial1.read();
      Serial0.write(data);
    }
  } else {
    static String receivedData;

    while (Serial0.available() > 0) {
      char receivedChar = Serial0.read();
      receivedData += receivedChar;

      // Detect the end of the JSON string based on a specific termination character
      if (receivedChar == '\n') {
        // Now we have received the complete JSON string
        DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
        if (err == DeserializationError::Ok) {
          if (echoMsgFlag) {
            msg(receivedData, false);
          }
          jsonCmdReceiveHandler(jsonCmdReceive);
        } else {
          // Handle JSON parsing error here
          msg("JSON parsing error: ", false);
          msg(err.c_str());
        }
        // Reset the receivedData for the next JSON string
        receivedData = "";
      }
    }
  } 
}


void loop() {
  // unsigned long startMicros = micros();
#ifndef TTL_ADAPTER_MACHINE
  serialCtrl();
#endif

  receiveCANasJson();
  // unsigned long endMicros = micros();
  // unsigned long elapsedTime = endMicros - startMicros;
  // Serial.println(elapsedTime);

#ifdef UART0_AS_SBUS
  if (sbus.Read()) {
    /* Grab the received data */
    sbusData = sbus.data();
    /* Display the received data */
    for (int8_t i = 0; i < sbusData.NUM_CH; i++) {
      Serial.print(sbusData.ch[i]);
      Serial.print("\t");
    }
    /* Display lost frames and failsafe data */
    Serial.print(sbusData.lost_frame);
    Serial.print("\t");
    Serial.println(sbusData.failsafe);

    float speed_limit = 1.0;
    if (sbusData.ch[4] < 300) {
      speed_limit = 0.33;
    } else if (sbusData.ch[4] == 1002) {
      speed_limit = 0.66;
    } else if (sbusData.ch[4] > 1700) {
      speed_limit = 1.0;
    }

    float speed_input = constrain(float(sbusData.ch[2] - SBUS_MID)/SBUS_RAN, -1.0, 1.0);
    float turn_input = - constrain(float(sbusData.ch[3] - SBUS_MID)/SBUS_RAN, -1.0, 1.0);

    float left_speed = (speed_input) * speed_limit * 6000.0 - (turn_input * 0.33 * 6000.0);
    float right_speed = (speed_input) * speed_limit * 6000.0 + (turn_input * 0.33 * 6000.0);

    jointsCtrl.hubMotorCtrl(left_speed, -(right_speed), right_speed, left_speed);
  }
#endif

#ifdef USE_ROBOTIC_ARM
  static unsigned long lastFeedbackTime = 0;
  if (jointsCtrl.linkArmFeedbackFlag && (millis() - lastFeedbackTime >= (1000 / jointsCtrl.linkArmFeedbackHz))) {
    lastFeedbackTime = millis();
    memcpy(jointFBRads, jointsCtrl.getJointFBRads(), sizeof(jointFBRads));
    memcpy(jointFBTorques, jointsCtrl.getLinkArmTorqueSC(), sizeof(jointFBTorques));
    jsonFeedback.clear();
    jsonFeedback["T"] = -CMD_GET_JOINT_FB_RADS;
    for (int i = 0; i < JOINTS_NUM; i++) {
      jsonFeedback["rads"][i] = String(jointFBRads[i], 3).toDouble();
      jsonFeedback["tors"][i] = jointFBTorques[i];
    }
    if (jointsCtrl.sbusCtrl) {
      jointsCtrl.readSBUS();
      if (jointsCtrl.sbus[8] == SBUS_MAX) {
        jointsCtrl.fpv_r = 260.5;
        jointsCtrl.fpv_b = 0;
        jointsCtrl.fpv_z = 122.38;
        jointsCtrl.fpv_g = 0;
        jointsCtrl.linkArmFPVIK(260.5, 0, 122.38, 0);
      } else if (jointsCtrl.sbus[7] == SBUS_MAX) {
        jointsCtrl.torqueLock(254, 0);
      } else {
        if (jointsCtrl.sbus[0] == 0) {
          jointsCtrl.fpv_z += (SBUS_MID - jointsCtrl.sbus[2])*0.0036;
          jointsCtrl.fpv_b += (jointsCtrl.sbus[1] - SBUS_MID)*0.000018;
          jointsCtrl.fpv_r = jointsCtrl.mapDouble(jointsCtrl.sbus[10], SBUS_MAX, SBUS_MIN, 0, 320);
          if (jointsCtrl.sbus[6] == SBUS_MIN) {
            // jointsCtrl.fpv_g = 15;
            jointsCtrl.torqueLock(SERVO_ARRAY_3, 0);
          } else if (jointsCtrl.sbus[6] == SBUS_MID) {
            jointsCtrl.fpv_g = -50;
          } else if (jointsCtrl.sbus[6] == SBUS_MAX) {
            jointsCtrl.fpv_g = 0;
          }

          jointsCtrl.linkArmFPVIK(jointsCtrl.fpv_r, jointsCtrl.fpv_b, jointsCtrl.fpv_z, jointsCtrl.fpv_g);

          float speed_limit = 1.0;
          if (jointsCtrl.sbus[5] == SBUS_MIN) {
            speed_limit = 0.33;
          } else if (jointsCtrl.sbus[5] == SBUS_MID) {
            speed_limit = 0.66;
          } else if (jointsCtrl.sbus[5] == SBUS_MAX) {
            speed_limit = 1.0;
          }

          double speed_input = constrain(float(jointsCtrl.sbus[3] - SBUS_MID)/SBUS_RAN, -1.0, 1.0);
          double turn_input = - constrain(float(jointsCtrl.sbus[4] - SBUS_MID)/SBUS_RAN, -1.0, 1.0);

          int left_speed = (speed_input) * speed_limit * 6000.0 - (turn_input * 0.33 * 6000.0);
          int right_speed = (speed_input) * speed_limit * 6000.0 + (turn_input * 0.33 * 6000.0);

          delay(2);
          jointsCtrl.hubMotorCtrl(left_speed, right_speed, right_speed, left_speed);
          delay(2);
        } else {
          jointsCtrl.hubMotorCtrl(0, 0, 0, 0);
          jointsCtrl.torqueLock(254, 0);
        }
      }
    }

    serializeJson(jsonFeedback, outputString);
    msg(outputString);
    if (jointsCtrl.espnowLeader && !jointsCtrl.sbusCtrl) {
      jsonFeedback.clear();
      jsonFeedback["T"] = CMD_LINK_ARM_SC_JOINTS_CTRL_RAD;
      jsonFeedback["rad"][0] = jointFBRads[0];
      jsonFeedback["rad"][1] = jointFBRads[1];
      jsonFeedback["rad"][2] = jointFBRads[2];
      jsonFeedback["rad"][3] = jointFBRads[3];
      wireless.sendEspNowJson(broadcastAddress, jsonFeedback);
    }
  }
#endif

  char cmdBuffer[CMD_MAX_LEN];
  if (xQueueReceive(cmdQueue, cmdBuffer, 0) == pdPASS) {
    DeserializationError err = deserializeJson(jsonCmdReceive, cmdBuffer);
    if (err == DeserializationError::Ok) {
      jsonCmdReceiveHandler(jsonCmdReceive);
      jsonCmdReceive.clear();
    } else {
      msg("JSON parse error");
    }
  }

  pushTelemetry();

#ifdef CAN_BUS_MACHINE
  canTestMachine();
#endif

#ifdef TTL_ADAPTER_MACHINE
  msg("TTL Adapter Test");
  jointsCtrl.ttlTestMachine();
#endif
}

