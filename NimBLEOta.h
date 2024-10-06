#ifndef NIMBLE_OTA_H_
#define NIMBLE_OTA_H_

#include "esp_ota_ops.h"

class NimBLEServer;
class NimBLECharacteristicCallbacks;
class NimBLEUUID;
struct ble_gap_conn_desc;

class NimBLEOta{
public:
    NimBLEServer* createServer();
    NimBLEUUID getServiceUUID() const;
    void abortOta();
    bool isInProgress() const { return m_inProgess; };
private:
    void setInProgress(bool inProgress) { m_inProgess = inProgress; };
    class NimBLEOtaCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    public:
        NimBLEOtaCharacteristicCallbacks(NimBLEOta* pNimBLEOta) : m_pOta(pNimBLEOta) {}
        void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override;
        void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override;
        void commandOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc);
        void firmwareOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc);
    private:
        NimBLEOta* m_pOta{nullptr};
    };

    NimBLEOta::NimBLEOtaCharacteristicCallbacks m_charCallbacks{this};
    uint32_t          m_fileLen{};
    uint8_t*          m_pBuf{nullptr};
    bool              m_inProgess{false};
    uint32_t          m_sector{};
    uint32_t          m_packet{};
    uint32_t          m_offset{};
    uint32_t          m_recvLen{};
    ble_gap_conn_desc m_client{};
    esp_ota_handle_t  m_writeHandle{};
    esp_partition_t   m_partition{};
};

#endif // NIMBLE_OTA_H_