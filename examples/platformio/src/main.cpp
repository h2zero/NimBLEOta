#include "NimBLEDevice.h"
#include "NimBLEOta.h"

class NimBleOtaServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer)
    {
        Serial.print("Client connected");
    }

    void onDisconnect(NimBLEServer *pServer)
    {
        Serial.print("Client disconnected");
    }
};

void setup()
{
    Serial.begin(115200);
    NimBLEDevice::init("NIMBLE OTA");
    NimBLEDevice::setMTU(517);
    NimBLEServer* pServer = NimBLEOta::createServer();
    pServer->setCallbacks(new NimBleOtaServerCallbacks());
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(NimBLEOta::getServiceUUID());
    pAdvertising->start();
}

void loop()
{
    delay(2000);
}
