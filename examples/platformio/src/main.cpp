#include "NimBLEDevice.h"
#include "NimBLEOta.h"

NimBLEOta bleOta;

class NimBleOtaServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer)
    {
        Serial.println("Client connected");
    }

    void onDisconnect(NimBLEServer *pServer)
    {
        Serial.println("Client disconnected");
    }
};

void setup()
{
    Serial.begin(115200);
    NimBLEDevice::init("NIMBLE OTA");
    NimBLEDevice::setMTU(517);
    NimBLEServer* pServer = bleOta.createServer();
    pServer->setCallbacks(new NimBleOtaServerCallbacks());
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(bleOta.getServiceUUID());
    pAdvertising->start();
    Serial.println("OTA service started");
}

void loop()
{
    delay(2000);
}
