// Copyright 2025 Ryan Powell and NimBLEOta contributors
// Sponsored by Theengs https://www.theengs.io
// MIT License

#ifndef NIMBLE_OTA_H_
#define NIMBLE_OTA_H_

#include <esp_ota_ops.h>
#include <NimBLEAddress.h>
#include <NimBLECharacteristic.h>

class NimBLEOtaCallbacks;
struct ble_npl_callout;

/**
 * @brief A model of the BLE OTA Service
 */
class NimBLEOta {
  public:
    NimBLEOta() = default;
    NimBLEService* start(NimBLEOtaCallbacks* pCallbacks = nullptr, bool secure = false);
    void           abortUpdate();
    bool           startAbortTimer(uint32_t seconds);
    void           stopAbortTimer();
    bool           isInProgress() const { return m_inProgress; };
    NimBLEUUID     getServiceUUID() const;

    enum Reason {
        StartCmd,
        StopCmd,
        Disconnected,
        Reconnected,
        FlashError,
        LengthError,
    };

  private:
    static void abortTimerCb(ble_npl_event* event);

    class NimBLEOtaCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
      public:
        NimBLEOtaCharacteristicCallbacks(NimBLEOta* pNimBLEOta) : m_pOta(pNimBLEOta) {}
        void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
        void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override;
        void commandOnWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo);
        void firmwareOnWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo);

      private:
        uint16_t   getCrc16(const uint8_t* buf, int len);
        NimBLEOta* m_pOta{nullptr};
    } m_charCallbacks{this};

    NimBLEOtaCallbacks* m_pCallbacks{nullptr};
    ble_npl_callout     m_otaCallout{};
    NimBLEAddress       m_clientAddr{};
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