#ifndef NIMBLE_OTA_H_
# define NIMBLE_OTA_H_

# include "esp_ota_ops.h"

class  NimBLEService;
class  NimBLECharacteristicCallbacks;
class  NimBLEUUID;
class  NimBLEOtaCallbacks;
struct ble_gap_conn_desc;
struct ble_npl_callout;

class NimBLEOta {
public:
    NimBLEService* start(NimBLEOtaCallbacks* pCallbacks = nullptr);
    NimBLEUUID     getServiceUUID() const;
    void           abortUpdate();
    bool           startAbortTimer(uint32_t seconds);
    void           stopAbortTimer();
    bool           isInProgress() const { return m_inProgress; };

    enum Reason {
        StartCmd,
        StopCmd,
        Disconnected,
        Reconnected,
        FlashError,
        LengthError,
    };

private:
    static void abortTimerCb(ble_npl_event *event);

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
    } m_charCallbacks{this};

    NimBLEOtaCallbacks* m_pCallbacks{nullptr};
    ble_npl_callout     m_otaCallout{};
    ble_gap_conn_desc   m_client{};
    esp_ota_handle_t    m_writeHandle{};
    esp_partition_t     m_partition{};
    uint32_t            m_fileLen{};
    uint32_t            m_recvLen{};
    uint8_t*            m_pBuf{nullptr};
    uint16_t            m_sector{};
    uint16_t            m_offset{};
    uint8_t             m_packet{};
    bool                m_inProgress{false};
};

class NimBLEOtaCallbacks {
public:
    virtual ~NimBLEOtaCallbacks() = default;
    virtual void onStart(NimBLEOta* ota, uint32_t firmwareSize, NimBLEOta::Reason reason);
    virtual void onProgress(NimBLEOta* ota, uint32_t current, uint32_t total);
    virtual void onStop(NimBLEOta* ota, NimBLEOta::Reason reason);
    virtual void onComplete(NimBLEOta* ota);
    virtual void onError(NimBLEOta* ota, esp_err_t err, NimBLEOta::Reason reason);
};

#endif // NIMBLE_OTA_H_