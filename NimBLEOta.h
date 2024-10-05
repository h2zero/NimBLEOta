#ifndef NIMBLE_OTA_H_
#define NIMBLE_OTA_H_

class NimBLEServer;

class NimBLEOta
{
public:
    static NimBLEServer *createServer();
    static NimBLEUUID getServiceUUID();
    static void abortOta();
};

#endif // NIMBLE_OTA_H_