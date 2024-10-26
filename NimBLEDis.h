#ifndef NIMBLE_DIS_H_
#define NIMBLE_DIS_H_

class NimBLEService;
class NimBLEUUID;

class NimBLEDis {
public:
    bool init();
    bool start();
    bool setModelNumber(const char* value);
    bool setSerialNumber(const char* value);
    bool setFirmwareRevision(const char* value);
    bool setHardwareRevision(const char* value);
    bool setSoftwareRevision(const char* value);
    bool setManufacturerName(const char* value);
    bool setSystemId(const char* value);
    bool setPnp(uint8_t src, uint16_t vid, uint16_t pid, uint16_t ver);

private:
    bool createDisChar(const NimBLEUUID& uuid, const uint8_t* value, uint16_t length);
    NimBLEService* m_pDisService{nullptr};
};

#endif // NIMBLE_DIS_H_