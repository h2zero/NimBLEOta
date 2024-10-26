#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEOta.h>

NimBLEOta bleOta;

class NimBleOtaServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer) override {
        Serial.println("Client connected");
    }

    void onDisconnect(NimBLEServer *pServer) override {
        Serial.println("Client disconnected");
    }
} bleOtaServerCallbacks{};

class OtaCallbacks : public NimBLEOtaCallbacks {
    void onStart(NimBLEOta* ota, uint32_t firmwareSize, NimBLEOta::Reason reason) override {
        if (reason == NimBLEOta::StartCmd) {
            Serial.printf("Start OTA with firmware size: %d\n", firmwareSize);
            return;
        }

        if (reason == NimBLEOta::Reconnected) {
            Serial.println("Reconnected, resuming OTA Update");
            ota->stopAbortTimer();
            return;
        }

        Serial.printf("OTA start, firmware size: %d, Reason: %u\n", firmwareSize, reason);
    }

    void onProgress(NimBLEOta* ota, uint32_t current, uint32_t total) override {
        Serial.printf("OTA progress: %.1f%%, cur: %u, tot: %u\n", static_cast<float>(current) / total * 100, current, total);
    }

    void onStop(NimBLEOta* ota, NimBLEOta::Reason reason) override {
        if (reason == NimBLEOta::Disconnected) {
            Serial.println("OTA stopped, client disconnected");
            ota->startAbortTimer(30); // abort if client does not restart ota in 30 seconds
            return;
        }

        if (reason == NimBLEOta::StopCmd) {
            Serial.println("OTA stopped by command - aborting");
            ota->abortUpdate();
            return;
        }

        Serial.printf("OTA stopped, Reason: %u\n", reason);
    }

    void onComplete(NimBLEOta* ota) override {
        Serial.println("OTA update complete - restarting in 2 seconds");
        delay(2000);
        ESP.restart();
    }

    void onError(NimBLEOta* ota, esp_err_t err, NimBLEOta::Reason reason) override {
        Serial.printf("OTA error: %d\n", err);
        if (reason == NimBLEOta::FlashError) {
            Serial.println("Flash error, aborting OTA update");
            ota->abortUpdate();
        }
    }
} otaCallbacks{};

void setup() {
    Serial.begin(115200);
    NimBLEDevice::init("NIMBLE OTA");
    NimBLEDevice::setMTU(517);
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&bleOtaServerCallbacks);
    bleOta.start(&otaCallbacks);
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(bleOta.getServiceUUID());
    pAdvertising->start();
    Serial.println("NimBLEOta started");
}

void loop() {
    delay(2000);
}
