#include "FilesCtrl.h"
#include <nvs_flash.h>
#include <esp_system.h>
#include "FS.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

JsonDocument cmdJson;

void FilesCtrl::init() {
    if (!flashStatus) {
        if (nvs_flash_init() == ESP_OK) {
            Serial.println("nvs_flash_init() success");
            Serial0.println("nvs_flash_init() success");
        } else {
            Serial.println("nvs_flash_init() failed");
            Serial0.println("nvs_flash_init() failed");
        }
        if (LittleFS.begin()) {
            Serial.println("LittleFS.begin() success");
            Serial0.println("LittleFS.begin() success");
        } else {
            Serial.println("LittleFS.begin() failed");
            Serial0.println("LittleFS.begin() failed");
            Serial.println("LittleFS.begin() failed. Formatting...");
            Serial0.println("LittleFS.begin() failed. Formatting...");
            if (LittleFS.format()) {
                Serial.println("Flash formatted successfully.");
                Serial0.println("Flash formatted successfully.");
                if (LittleFS.begin()) {
                  Serial.println("LittleFS initialized successfully after formatting.");
                  Serial0.println("LittleFS initialized successfully after formatting.");
                } else {
                  Serial.println("LittleFS initialization failed after formatting.");
                  Serial0.println("LittleFS initialization failed after formatting.");
                }
            } else {
                Serial.println("Flash formatting failed.");
                Serial0.println("Flash formatting failed.");
            }
        }
        flashStatus = true;
    }
}

void FilesCtrl::flash() {
    if (flashStatus) {
        LittleFS.format();
        Serial.println("=== LittleFS formatted ===");
        Serial0.println("=== LittleFS formatted ===");
    }
}

void FilesCtrl::scan() {
    if (flashStatus) {
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        Serial.println("=== Scaning Files ===");
        Serial0.println("=== Scaning Files ===");
        while (file) {
            Serial.print("file name: ");
            Serial.print(file.name());
            Serial.print(" file size: ");
            Serial.println(file.size());
            Serial0.print("file name: ");
            Serial0.print(file.name());
            Serial0.print(" file size: ");
            Serial0.println(file.size());
            file = root.openNextFile();
        }
    }
}

void FilesCtrl::createMission(String missionName, String contentInput) {
    if (flashStatus) {
        if (LittleFS.exists("/" + missionName + ".mission")) {
            Serial.println("file already exists");
            Serial0.println("file already exists");
            return;
        }
        File createFile = LittleFS.open("/" + missionName + ".mission", FILE_WRITE);
        createFile.close();
        File file = LittleFS.open("/" + missionName + ".mission", FILE_APPEND);
        if (file) {
            if(file.println(contentInput) == 0){
                Serial.println("file write failed");
                Serial0.println("file write failed");
            } else {
                Serial.println("file write succeed");
                Serial0.println("file write succeed");
            }
            file.close();
            Serial.println("file created");
            Serial0.println("file created");
        } else {
            Serial.println("file creation failed");
            Serial0.println("file creation failed");
        }
    }
}

void FilesCtrl::missionContent(String missionName) {
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if (!file) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return;
    }
    String firstLine = file.readStringUntil('\n');
    Serial.println("=== File Content ===");
    Serial.print("File Name: ");
    Serial.println(missionName);
    Serial.print("Intro: ");
    Serial.println(firstLine);
    Serial0.println("=== File Content ===");
    Serial0.print("File Name: ");
    Serial0.println(missionName);
    Serial0.print("Intro: ");
    Serial0.println(firstLine);
    int _lineNum = 0;
    while (file.available()) {
		_lineNum++;
		String line = file.readStringUntil('\n');
		Serial.print("[StepNum: ");Serial.print(_lineNum);Serial.print(" ] - ");
		Serial.println(line);
        Serial0.print("[StepNum: ");Serial0.print(_lineNum);Serial0.print(" ] - ");
		Serial0.println(line);
    }
    file.close();
}

void FilesCtrl::appendStep(String missionName, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return;
    }
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
	if (err == DeserializationError::Ok) {
		Serial.println("json parsing succeed.");
        Serial0.println("json parsing succeed.");
        File file = LittleFS.open("/" + missionName + ".mission", "a");
        if(!file){
            Serial.println("Error opening file for appending.");
            Serial0.println("Error opening file for appending.");
            return;
        }
        file.println(jsonCmd);
        file.close();
        Serial.println("=== Edited File ===");
        Serial0.println("=== Edited File ===");
        FilesCtrl::missionContent(missionName);
		cmdJson.clear();
	} else {
		cmdJson.clear();
		Serial.println("[deserializeJson err]");
        Serial0.println("[deserializeJson err]");
	}
}

void FilesCtrl::insertStep(String missionName, int stepNum, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return;
    }
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
    if (err == DeserializationError::Ok) {
        Serial.println("json parsing succeed.");
        Serial0.println("json parsing succeed.");
        File file = LittleFS.open("/" + missionName + ".mission", "r");
        if(!file){
        Serial.println("Error opening file for inserting.");
        Serial0.println("Error opening file for inserting.");
        return;
        }
        String _content = "";
        _content += file.readStringUntil('\n') + "\n";
        for (int i = 0; i < stepNum - 1; i++) {
            _content += file.readStringUntil('\n') + "\n";
        }
        _content += jsonCmd + "\n";
        while (file.available()) {
            _content += file.readStringUntil('\n') + "\n";
        }
        file.close();
        file = LittleFS.open("/" + missionName + ".mission", "w");
        if(!file){
        Serial.println("Error opening file for inserting.");
        Serial0.println("Error opening file for inserting.");
        return;
        }
        file.print(_content);
        file.close();
        Serial.println("=== Edited File ===");
        Serial0.println("=== Edited File ===");
        FilesCtrl::missionContent(missionName);
        cmdJson.clear();
    } else {
        cmdJson.clear();
        Serial.println("[deserializeJson err]");
        Serial0.println("[deserializeJson err]");
    }
}

void FilesCtrl::replaceStep(String missionName, int stepNum, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return;
    }
    if (stepNum < 1) {
        Serial.println("stepNum should be greater than 0.");
        Serial0.println("stepNum should be greater than 0.");
        return;
    }
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
    if (err == DeserializationError::Ok) {
        Serial.println("json parsing succeed.");
        Serial0.println("json parsing succeed.");
        File file = LittleFS.open("/" + missionName + ".mission", "r");
        if(!file){
        Serial.println("Error opening file for replacing.");
        Serial0.println("Error opening file for replacing.");
        return;
        }
        String _content = "";
        _content += file.readStringUntil('\n') + "\n";
        for (int i = 0; i < stepNum - 1; i++) {
            _content += file.readStringUntil('\n') + "\n";
        }
        _content += jsonCmd + "\n";
        file.readStringUntil('\n');
        while (file.available()) {
            _content += file.readStringUntil('\n') + "\n";
        }
        file.close();
        file = LittleFS.open("/" + missionName + ".mission", "w");
        if(!file){
        Serial.println("Error opening file for replacing.");
        Serial0.println("Error opening file for replacing.");
        return;
        }
        file.print(_content);
        file.close();
        Serial.println("=== Edited File ===");
        Serial0.println("=== Edited File ===");
        FilesCtrl::missionContent(missionName);
        cmdJson.clear();
    } else {
        cmdJson.clear();
        Serial.println("[deserializeJson err]");
        Serial0.println("[deserializeJson err]");
    }
}

void FilesCtrl::deleteStep(String missionName, int stepNum) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return;
    }
    if (stepNum < 1) {
        Serial.println("stepNum should be greater than 0.");
        Serial0.println("stepNum should be greater than 0.");
        return;
    }
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if(!file){
        Serial.println("Error opening file for deleting.");
        Serial0.println("Error opening file for deleting.");
        return;
    }
    String _content = "";
    _content += file.readStringUntil('\n') + "\n";
    for (int i = 0; i < stepNum - 1; i++) {
        _content += file.readStringUntil('\n') + "\n";
    }
    file.readStringUntil('\n');
    while (file.available()) {
        _content += file.readStringUntil('\n') + "\n";
    }
    file.close();
    file = LittleFS.open("/" + missionName + ".mission", "w");
    if(!file){
        Serial.println("Error opening file for deleting.");
        Serial0.println("Error opening file for deleting.");
        return;
    }
    file.print(_content);
    file.close();
    Serial.println("=== Edited File ===");
    Serial0.println("=== Edited File ===");
    FilesCtrl::missionContent(missionName);
}

String FilesCtrl::readStep(String missionName, int stepNum) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return "";
    }
    if (stepNum < 1) {
        Serial.println("stepNum should be greater than 0.");
        Serial0.println("stepNum should be greater than 0.");
        return "";
    }
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if(!file){
        Serial.println("Error opening file for reading.");
        Serial0.println("Error opening file for reading.");
        return "";
    }
    String _content = "";
    file.readStringUntil('\n');
    for (int i = 0; i < stepNum - 1; i++) {
        file.readStringUntil('\n');
    }
    _content += file.readStringUntil('\n');
    file.close();
    return _content;
}

void FilesCtrl::deleteMission(String missionName) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return;
    }
    LittleFS.remove("/" + missionName + ".mission");
    Serial.println("file deleted.");
    Serial0.println("file deleted.");
}

bool FilesCtrl::checkMission(String missionName) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        return false;
    }
    return true;
}

bool FilesCtrl::checkStepByType(String missionName, int cmdType) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        return false;
    }
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if(!file){
        Serial.println("Error opening file for reading.");
        return false;
    }
    file.readStringUntil('\n');
    while (file.available()) {
        String _content = file.readStringUntil('\n');
        DeserializationError err = deserializeJson(cmdJson, _content);
        if (err == DeserializationError::Ok) {
            if (cmdJson["T"].as<int>() == cmdType) {
                cmdJson.clear();
                return true;
            }
        }
        cmdJson.clear();
    }
    return false;
}

bool FilesCtrl::checkReplaceStep(String missionName, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return false;
    }

    cmdJson.clear();
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
    if (err != DeserializationError::Ok) {
        Serial.println("[deserializeJson err]");
        Serial0.println("[deserializeJson err]");
        return false;
    }

    if (!cmdJson["T"].is<int>()) {
        Serial.println("Invalid jsonCmd: missing 'T' field.");
        Serial0.println("Invalid jsonCmd: missing 'T' field.");
        return false;
    }
    int targetT = cmdJson["T"].as<int>();

    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if (!file) {
        Serial.println("Error opening file for reading.");
        Serial0.println("Error opening file for reading.");
        return false;
    }

    String _content = "";
    String line;
    bool replaced = false;
    _content += file.readStringUntil('\n') + "\n";

    // Read and process each line
    while (file.available()) {
        line = file.readStringUntil('\n');
        cmdJson.clear();
        DeserializationError lineErr = deserializeJson(cmdJson, line);

        if (lineErr == DeserializationError::Ok && cmdJson["T"].as<int>() == targetT) {
            replaced = true; // Mark as replaced for each matching line
        } else {
            _content += line + "\n"; // Keep the line if not matching
            Serial.print("read: ");
            Serial.println(cmdJson["T"].as<int>());
            Serial.print("target: ");
            Serial.println(targetT);
            Serial0.print("read: ");
            Serial0.println(cmdJson["T"].as<int>());
            Serial0.print("target: ");
            Serial0.println(targetT);
        }
    }
    file.close();

    // Write the updated content back to the file
    file = LittleFS.open("/" + missionName + ".mission", "w");
    if (!file) {
        Serial.println("Error opening file for writing.");
        Serial0.println("Error opening file for writing.");
        return false;
    }
    file.print(_content);
    file.close();

    // Append the new jsonCmd if replacement occurred
    if (replaced) {
        appendStep(missionName, jsonCmd);
        Serial.println("Step replaced successfully.");
        Serial0.println("Step replaced successfully.");
        return true;
    } else {
        appendStep(missionName, jsonCmd);
        Serial.println("No matching step found to replace. Appended instead.");
        Serial0.println("No matching step found to replace. Appended instead.");
        return true;
    }
}

String FilesCtrl::findCmdByType(String missionName, int cmdType) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        Serial0.println("file not found.");
        return "";
    }

    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if (!file) {
        Serial.println("Error opening file for reading.");
        Serial0.println("Error opening file for reading.");
        return "";
    }

    String line;
    String targetCmd = "";
    while (file.available()) {
        line = file.readStringUntil('\n');
        DeserializationError lineErr = deserializeJson(cmdJson, line);
        if (lineErr == DeserializationError::Ok && cmdJson["T"] == cmdType) {
            targetCmd = line;
            break;
        }
    }
    file.close();

    return targetCmd;
}

String FilesCtrl::getValueByMissionNameAndKey(String missionName, int cmdType, String keyName) {
    String keyValue;
    String targetCmd = findCmdByType(missionName, cmdType);
    DeserializationError lineErr = deserializeJson(cmdJson, targetCmd);
    if (lineErr == DeserializationError::Ok) {
        keyValue = cmdJson[keyName].as<const char*>();
    }
    return keyValue;
}