#ifndef NIMBLE_OTA_H_
#define NIMBLE_OTA_H_

#include "esp_ota_ops.h"

class NimBLEService;
class NimBLECharacteristicCallbacks;
class NimBLEUUID;
struct ble_gap_conn_desc;

class NimBLEOta{
public:
    NimBLEService* start();
    NimBLEUUID getServiceUUID() const;
    void abortUpdate();
    bool isInProgress() const { return m_inProgress; };
private:
    class NimBLEOtaCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    public:
        NimBLEOtaCharacteristicCallbacks(NimBLEOta* pNimBLEOta) : m_pOta(pNimBLEOta) {}
        void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override;
        void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override;
        void commandOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc);
        void firmwareOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc);
    private:
        uint16_t getCrc16(const uint8_t *buf, int len);
        NimBLEOta* m_pOta{nullptr};
    };

    NimBLEOta::NimBLEOtaCharacteristicCallbacks m_charCallbacks{this};
    ble_gap_conn_desc m_client{};
    esp_ota_handle_t  m_writeHandle{};
    esp_partition_t   m_partition{};
    uint32_t          m_fileLen{};
    uint32_t          m_recvLen{};
    uint8_t*          m_pBuf{nullptr};
    uint16_t          m_sector{};
    uint16_t          m_offset{};
    uint8_t           m_packet{};
    bool              m_inProgress{false};
};

#endif // NIMBLE_OTA_H_