#include "NimBLEDevice.h"
#include "NimBLEOta.h"

#define BLE_OTA_SERVICE_UUID (uint16_t)0x8018
#define RECV_FW_UUID (uint16_t)0x8020
#define OTA_BAR_UUID (uint16_t)0x8021
#define COMMAND_UUID (uint16_t)0x8022
#define CUSTOMER_UUID (uint16_t)0x8023
#define OTA_BLOCK_SIZE 4098
#define OTA_IDX_NB 4
#define START_OTA_CMD 0x0001
#define STOP_OTA_CMD 0x0002
#define ACK_OTA_CMD 0x0003
#define OTA_ACCEPT 0x0000
#define OTA_REJECT 0x0001
#define SIGN_ERROR 0x0003
#define CRC_ERROR 0x0001
#define INDEX_ERROR 0x0002
#define OTA_FW_SUCCESS 0x0000
#define LEN_ERROR 0x0003
#define START_ERROR 0x0005
#define FW_ACK_LENGTH 20
#define CMD_ACK_LENGTH 20

static const char *LOG_TAG = "NimBLEOta";
static NimBLEOtaCallbacks defaultCallbacks;

extern "C" struct ble_npl_eventq *nimble_port_get_dflt_eventq(void);

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::firmwareOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
    if (!m_pOta->isInProgress()) {
        NIMBLE_LOGW(LOG_TAG, "ota not started");
        return;
    }

    if (NimBLEAddress(desc->peer_id_addr) != NimBLEAddress(m_pOta->m_client.peer_id_addr)) {
        NIMBLE_LOGW(LOG_TAG, "Received write from unknown client - ignored");
        return;
    }

    esp_err_t err = ESP_OK;
    const uint8_t *data = pCharacteristic->getValue();
    auto dataLen = pCharacteristic->getValue().length();
    uint8_t fwAck[FW_ACK_LENGTH]{};
    uint16_t otaResp = OTA_FW_SUCCESS;
    uint16_t crc = 0;
    uint32_t writeLen = 0;
    uint16_t recvSector = *reinterpret_cast<const uint16_t*>(data);

    fwAck[0] = data[0];
    fwAck[1] = data[1];

    if (recvSector != m_pOta->m_sector) {
        if (recvSector == 0xffff) {
            NIMBLE_LOGD(LOG_TAG, "Last sector received");
        } else {
            if (data[2] == 0xff) { // only send ack after last packet received due to write without response not waiting
                NIMBLE_LOGE(LOG_TAG, "Sector index error, expected: %u, received: %u", m_pOta->m_sector, recvSector);
                otaResp = INDEX_ERROR;
                goto SendAck;
            }

            return;
        }
    }

    if (data[2] != m_pOta->m_packet) {
        if (data[2] == 0xff) {
            NIMBLE_LOGD(LOG_TAG, "last packet");
            dataLen -= 2; // in the last packet the last 2 bytes are crc
        } else {
            // There is no response for out of sequence packet error, will fail crc or length check
            NIMBLE_LOGE(LOG_TAG, "packet sequence error, cur: %" PRIu32 ", recv: %d", m_pOta->m_packet, data[2]);
        }
    }

    memcpy(m_pOta->m_pBuf + m_pOta->m_offset, data + 3, dataLen - 3);
    m_pOta->m_offset += dataLen - 3;

    NIMBLE_LOGD(LOG_TAG, "Sector:%" PRIu32 ", total length:%" PRIu32 ", length:%d", m_pOta->m_sector, m_pOta->m_offset, dataLen - 3);
    if (data[2] != 0xff) { // not last packet
        NIMBLE_LOGD(LOG_TAG, "waiting for next packet");
        m_pOta->m_packet++;
        return;
    }

    writeLen = std::min<size_t>(OTA_BLOCK_SIZE - 2, m_pOta->m_offset);
    if ((recvSector != 0xffff && (m_pOta->m_recvLen + writeLen) != m_pOta->m_fileLen) && m_pOta->m_offset != OTA_BLOCK_SIZE - 2) {
        NIMBLE_LOGE(LOG_TAG, "sector length error, received: %d bytes", m_pOta->m_offset);
        otaResp = LEN_ERROR;
        goto SendAck;
    }

    crc = *reinterpret_cast<const uint16_t*>(data + dataLen);
    if (crc != getCrc16(m_pOta->m_pBuf , m_pOta->m_offset)) {
        NIMBLE_LOGE(LOG_TAG, "crc error");
        otaResp = CRC_ERROR;
        goto SendAck;
    }

    err = esp_ota_write(m_pOta->m_writeHandle, static_cast<const void*>(m_pOta->m_pBuf), writeLen);
    if (err != ESP_OK) {
        NIMBLE_LOGE(LOG_TAG, "esp_ota_write failed! err=0x%x", err);
        goto Done;
    }

    m_pOta->m_recvLen += writeLen;
    m_pOta->m_pCallbacks->onProgress(m_pOta, m_pOta->m_recvLen, m_pOta->m_fileLen);
    if (m_pOta->m_recvLen >= m_pOta->m_fileLen) {
        err = esp_ota_end(m_pOta->m_writeHandle);
        if (err != ESP_OK) {
            NIMBLE_LOGE(LOG_TAG, "esp_ota_end failed! err=0x%x", err);
            goto Done;
        }

        err = esp_ota_set_boot_partition(&m_pOta->m_partition);
        if (err != ESP_OK) {
            NIMBLE_LOGE(LOG_TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
            goto Done;
        }

    }

SendAck:
    m_pOta->m_packet = 0;
    m_pOta->m_offset = 0;
    fwAck[2] = otaResp;
    fwAck[3] = (otaResp >> 8) & 0xff;
    fwAck[4] = m_pOta->m_sector;
    fwAck[5] = (m_pOta->m_sector >> 8) & 0xff;
    crc = getCrc16(fwAck, 18);
    fwAck[18] = crc & 0xff;
    fwAck[19] = (crc & 0xff00) >> 8;
    pCharacteristic->setValue(fwAck, FW_ACK_LENGTH);
    pCharacteristic->indicate();

    if (otaResp == OTA_FW_SUCCESS) {
        m_pOta->m_sector++;
    }

    if (m_pOta->m_recvLen < m_pOta->m_fileLen) {
        return;
    }

Done:
    if (err == ESP_OK) {
        m_pOta->abortUpdate(); // Reset the OTA state
        m_pOta->m_pCallbacks->onComplete(m_pOta);
    } else {
        m_pOta->m_pCallbacks->onError(m_pOta, err, NimBLEOta::FlashError);
    }
}

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::commandOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
    if (NimBLEAddress(m_pOta->m_client.peer_id_addr) == NimBLEAddress("")) {
        m_pOta->m_client = *desc;
    } else if (NimBLEAddress(desc->peer_id_addr) != NimBLEAddress(m_pOta->m_client.peer_id_addr)) {
        NIMBLE_LOGW(LOG_TAG, "Received command from unknown client - ignored");
        return;
    }

    uint16_t crc = 0;
    uint8_t cmdAck[CMD_ACK_LENGTH]{};
    cmdAck[0] = ACK_OTA_CMD;
    cmdAck[1] = (ACK_OTA_CMD >> 8) & 0xff;
    cmdAck[4] = OTA_REJECT;
    cmdAck[5] = (OTA_REJECT >> 8) & 0xff;

    if (pCharacteristic->getValue().length() == 20) {
        const uint8_t *data = pCharacteristic->getValue();
        uint16_t cmd = data[0] | (data[1] << 8);
        crc = data[18] | (data[19] << 8);
        cmdAck[2] = data[0];
        cmdAck[3] = data[1];

        if (getCrc16(data, 18) != crc || (cmd != START_OTA_CMD && cmd != STOP_OTA_CMD)) {
            NIMBLE_LOGE(LOG_TAG, "command %s error", cmd == START_OTA_CMD || cmd == STOP_OTA_CMD ? "CRC" : "invalid");
        } else if (cmd == START_OTA_CMD) {
            if (m_pOta->isInProgress()) {
                if (*reinterpret_cast<const uint32_t*>(data + 2) == m_pOta->m_fileLen) {
                    NIMBLE_LOGW(LOG_TAG, "Ota resuming");
                    m_pOta->m_pCallbacks->onStart(m_pOta, m_pOta->m_fileLen, NimBLEOta::Reconnected);
                    cmdAck[4] = OTA_ACCEPT;
                    cmdAck[5] = (OTA_ACCEPT >> 8) & 0xff;
                } else {
                    NIMBLE_LOGE(LOG_TAG, "Ota command error, file length mismatch - aborting");
                    m_pOta->abortUpdate();
                    m_pOta->m_pCallbacks->onError(m_pOta, ESP_FAIL, NimBLEOta::LengthError);
                }
            } else {
                m_pOta->m_fileLen = *reinterpret_cast<const uint32_t*>(data + 2);
                m_pOta->m_pBuf = static_cast<uint8_t*>(malloc(OTA_BLOCK_SIZE * sizeof(uint8_t)));
                if (m_pOta->m_pBuf == nullptr) {
                    NIMBLE_LOGE(LOG_TAG, "%s -  malloc fail", __func__);
                    goto SendAck;
                } else {
                    memset(m_pOta->m_pBuf, 0x0, OTA_BLOCK_SIZE);
                }

                const esp_partition_t *partition_ptr = esp_ota_get_boot_partition();
                if (partition_ptr == NULL) {
                    NIMBLE_LOGE(LOG_TAG, "boot partition NULL!\r\n");
                    goto SendAck;
                }

                if (partition_ptr->type != ESP_PARTITION_TYPE_APP) {
                    NIMBLE_LOGE(LOG_TAG, "esp_current_partition->type != ESP_PARTITION_TYPE_APP\r\n");
                    goto SendAck;
                }

                if (partition_ptr->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
                    m_pOta->m_partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
                } else {
                    const esp_partition_t *next_partition = esp_ota_get_next_update_partition(partition_ptr);
                    if (next_partition) {
                        m_pOta->m_partition.subtype = next_partition->subtype;
                    } else {
                        m_pOta->m_partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
                    }
                }
                m_pOta->m_partition.type = ESP_PARTITION_TYPE_APP;

                partition_ptr = esp_partition_find_first(m_pOta->m_partition.type, m_pOta->m_partition.subtype, NULL);
                if (partition_ptr == nullptr) {
                    NIMBLE_LOGE(LOG_TAG, "partition NULL!\r\n");
                    goto SendAck;
                }

                memcpy(&m_pOta->m_partition, partition_ptr, sizeof(esp_partition_t));
                if (esp_ota_begin(&m_pOta->m_partition, OTA_SIZE_UNKNOWN, &m_pOta->m_writeHandle) != ESP_OK) {
                    NIMBLE_LOGE(LOG_TAG, "esp_ota_begin failed!\r\n");
                    goto SendAck;
                }

                cmdAck[4] = OTA_ACCEPT;
                cmdAck[5] = (OTA_ACCEPT >> 8) & 0xff;
                m_pOta->m_inProgress = true;
                m_pOta->m_pCallbacks->onStart(m_pOta, m_pOta->m_fileLen, NimBLEOta::StartCmd);
            }
        } else if (cmd == STOP_OTA_CMD) {
            if (!m_pOta->isInProgress()) {
                NIMBLE_LOGW(LOG_TAG, "ota not started");
            } else {
                cmdAck[4] = OTA_ACCEPT;
                cmdAck[5] = (OTA_ACCEPT >> 8) & 0xff;
                m_pOta->m_pCallbacks->onStop(m_pOta, NimBLEOta::StopCmd);
            }
        } else {
            NIMBLE_LOGE(LOG_TAG, "Unknown Command");
        }
    } else {
        NIMBLE_LOGE(LOG_TAG, "command length error");
    }

SendAck:
    crc = getCrc16(cmdAck, 18);
    cmdAck[18] = crc;
    cmdAck[19] = (crc >> 8) & 0xff;
    pCharacteristic->setValue(cmdAck, CMD_ACK_LENGTH);
    pCharacteristic->indicate();
}

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t subValue) {
    NIMBLE_LOGI(LOG_TAG, "Ota client conn_handle: %d, subscribed: %s", desc->conn_handle, subValue ? "true" : "false");
    if (m_pOta->isInProgress() &&
        NimBLEAddress(m_pOta->m_client.peer_id_addr) == NimBLEAddress(desc->peer_id_addr) &&
        pChar->getUUID().equals(COMMAND_UUID)) {
        if (!subValue) { // client disconnected
            m_pOta->m_pCallbacks->onStop(m_pOta, NimBLEOta::Disconnected);
        }
    }
}

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::onWrite(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc) {
    if (pChar->getUUID().equals(COMMAND_UUID)) {
        commandOnWrite(pChar, desc);
    } else if (pChar->getUUID().equals(RECV_FW_UUID)) {
        firmwareOnWrite(pChar, desc);
    }
}

NimBLEService* NimBLEOta::start(NimBLEOtaCallbacks* pCallbacks) {
    m_pCallbacks = pCallbacks;
    if (m_pCallbacks == nullptr) {
        m_pCallbacks = &defaultCallbacks;
    }

    ble_npl_callout_init(&m_otaCallout, nimble_port_get_dflt_eventq(), NimBLEOta::abortTimerCb, this);

    NimBLEService *pService = NimBLEDevice::createServer()->createService(BLE_OTA_SERVICE_UUID);

    NimBLECharacteristic *pRecvFwCharacteristic = pService->createCharacteristic(RECV_FW_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pRecvFwCharacteristic->setCallbacks(&m_charCallbacks);

    NimBLECharacteristic *pCommandCharacteristic = pService->createCharacteristic(COMMAND_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pCommandCharacteristic->setCallbacks(&m_charCallbacks);

    /* TODO Customer Characteristic
    NimBLECharacteristic *pCustomerCharacteristic = pService->createCharacteristic(CUSTOMER_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pCustomerCharacteristic->setCallbacks(&m_charCallbacks);
    */

    /* TODO OTA Bar Characteristic
    NimBLECharacteristic *pOtaBarCharacteristic = pService->createCharacteristic(OTA_BAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pOtaBarCharacteristic->setCallbacks(&m_charCallbacks);
    */

    pService->start();
    return pService;
}

NimBLEUUID NimBLEOta::getServiceUUID() const {
    return NimBLEUUID{BLE_OTA_SERVICE_UUID};
}

void NimBLEOta::abortUpdate() {
    if (m_pBuf != nullptr) {
        free(m_pBuf);
        m_pBuf = nullptr;
    }

    m_recvLen = 0;
    m_offset = 0;
    m_packet = 0;
    m_sector = 0;
    m_fileLen = 0;
    m_client = ble_gap_conn_desc{};
    m_inProgress = false;
    esp_ota_abort(m_writeHandle);
}

bool NimBLEOta::startAbortTimer(uint32_t seconds) {
    ble_npl_time_t ticks;
    ble_npl_time_ms_to_ticks(seconds * 1000, &ticks);
    return ble_npl_callout_reset(&m_otaCallout, ticks) == BLE_NPL_OK;
}

void NimBLEOta::stopAbortTimer() {
    ble_npl_callout_stop(&m_otaCallout);
}

void NimBLEOta::abortTimerCb(ble_npl_event* event) {
    NimBLEOta* pOta = static_cast<NimBLEOta*>(event->arg);
    NIMBLE_LOGW(LOG_TAG, "Abort timer expired: aborting update!");
    pOta->abortUpdate();
}

uint16_t NimBLEOta::NimBLEOtaCharacteristicCallbacks::getCrc16(const uint8_t *buf, int len) {
    uint16_t crc = 0;
    int32_t i;

    while (len--) {
        crc ^= *buf++ << 8;

        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

static const char* CB_LOG_TAG = "Default-NimBLEOtaCallbacks";

void NimBLEOtaCallbacks::onStart(NimBLEOta* ota, uint32_t firmwareSize, NimBLEOta::Reason reason) {
    NIMBLE_LOGI(CB_LOG_TAG, "OTA started, firmware size: %" PRIu32, "Reason: %u", firmwareSize, reason);
}

void NimBLEOtaCallbacks::onProgress(NimBLEOta* ota, uint32_t current, uint32_t total) {
    NIMBLE_LOGI(CB_LOG_TAG, "OTA progress: %.f%%", current / total * 100.f);
}

void NimBLEOtaCallbacks::onStop(NimBLEOta* ota, NimBLEOta::Reason reason) {
    NIMBLE_LOGI(CB_LOG_TAG, "OTA stopped, Reason: %u - aborting", reason);
    ota->abortUpdate();
}

void NimBLEOtaCallbacks::onComplete(NimBLEOta* ota) {
    NIMBLE_LOGI(CB_LOG_TAG, "OTA complete - restarting in 2 seconds");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
}

void NimBLEOtaCallbacks::onError(NimBLEOta* ota, esp_err_t err, NimBLEOta::Reason reason) {
    NIMBLE_LOGE(CB_LOG_TAG, "OTA error: 0x%x, Reason: %u - aborting", err, reason);
    ota->abortUpdate();
}