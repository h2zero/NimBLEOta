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

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::firmwareOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
    if (!m_pOta->isInProgress()) {
        NIMBLE_LOGE(LOG_TAG, "ota not started");
        return;
    }

    if (NimBLEAddress(desc->peer_id_addr) != NimBLEAddress(m_pOta->m_client.peer_id_addr)) {
        NIMBLE_LOGE(LOG_TAG, "Received write from unknown client - ignored");
        return;
    }

    const uint8_t *data = pCharacteristic->getValue();
    auto dataLen = pCharacteristic->getValue().length();
    uint8_t fwAck[FW_ACK_LENGTH]{};
    uint16_t otaResp = OTA_FW_SUCCESS;
    uint16_t crc = 0;
    uint32_t writeLen = 0;
    uint16_t recvSector = *(uint16_t*)data;

    fwAck[0] = data[0];
    fwAck[1] = data[1];

    if (recvSector != m_pOta->m_sector) {
        if (recvSector == 0xffff) {
            // last sector
            NIMBLE_LOGI(LOG_TAG, "Last sector received");
        } else {
            // sector error
            NIMBLE_LOGE(LOG_TAG, "sector index error, cur: %" PRIu32 ", recv: %d", m_pOta->m_sector, recvSector);
            otaResp = INDEX_ERROR;
            goto SendAck;
        }
    }

    if (data[2] != m_pOta->m_packet) {
        if (data[2] == 0xff) { // last packet
            NIMBLE_LOGD(LOG_TAG, "last packet");
            dataLen -= 2; // in the last packet the last 2 bytes are crc
        } else { // packet seq error
            // There is no response for out of sequence packet error, will fail crc check
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

    NIMBLE_LOGI(LOG_TAG, "received sector %" PRIu32, m_pOta->m_sector);
    writeLen = std::min<size_t>(OTA_BLOCK_SIZE - 2, m_pOta->m_offset);
    if ((recvSector != 0xffff && (m_pOta->m_recvLen + writeLen) != m_pOta->m_fileLen) && m_pOta->m_offset != OTA_BLOCK_SIZE - 2) {
        NIMBLE_LOGE(LOG_TAG, "sector length error, received: %d bytes", m_pOta->m_offset);
        otaResp = LEN_ERROR;
        goto SendAck;
    }

    crc = *(uint16_t*)(data + dataLen);
    if (crc != getCrc16(m_pOta->m_pBuf , m_pOta->m_offset)) {
        NIMBLE_LOGE(LOG_TAG, "crc error");
        otaResp = CRC_ERROR;
        goto SendAck;
    }

    if (esp_ota_write(m_pOta->m_writeHandle, (const void *)m_pOta->m_pBuf, writeLen) != ESP_OK) {
        NIMBLE_LOGE(LOG_TAG, "esp_ota_write failed!\r\n");
        goto Restart;
    }

    m_pOta->m_recvLen += writeLen;
    if (m_pOta->m_recvLen >= m_pOta->m_fileLen) {
        if (esp_ota_end(m_pOta->m_writeHandle) != ESP_OK) {
            NIMBLE_LOGE(LOG_TAG, "esp_ota_end failed!\r\n");
            goto Restart;
        }

        if (esp_ota_set_boot_partition(&m_pOta->m_partition) != ESP_OK) {
            NIMBLE_LOGE(LOG_TAG, "esp_ota_set_boot_partition failed!\r\n");
            goto Restart;
        }

        NIMBLE_LOGI(LOG_TAG, "OTA update complete");
        goto Restart;
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
    return;

Restart:
    ble_npl_time_t ticks;
    ble_npl_time_delay(ble_npl_time_ms_to_ticks(2000, &ticks));
    esp_restart();
}

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::commandOnWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
    if (NimBLEAddress(m_pOta->m_client.peer_id_addr) == NimBLEAddress("")) {
        m_pOta->m_client = *desc;
    } else if (NimBLEAddress(desc->peer_id_addr) != NimBLEAddress(m_pOta->m_client.peer_id_addr)) {
        NIMBLE_LOGE(LOG_TAG, "Received command from unknown client - ignored");
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
                NIMBLE_LOGE(LOG_TAG, "Ota already started");
            } else {
                m_pOta->m_fileLen = *(uint32_t*)(data + 2);
                NIMBLE_LOGI(LOG_TAG, "BLE Ota star command received, length = %" PRIu32 "", m_pOta->m_fileLen);

                m_pOta->m_pBuf = (uint8_t *)malloc(OTA_BLOCK_SIZE * sizeof(uint8_t));
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
                NIMBLE_LOGI(LOG_TAG, "ota start success");
            }
        } else if (cmd == STOP_OTA_CMD) {
            NIMBLE_LOGI(LOG_TAG, "recv ota stop cmd");
            if (!m_pOta->isInProgress()) {
                NIMBLE_LOGE(LOG_TAG, "ota not started");
            } else {
                m_pOta->abortUpdate();
                cmdAck[4] = OTA_ACCEPT;
                cmdAck[5] = (OTA_ACCEPT >> 8) & 0xff;
            }
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

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
    NIMBLE_LOGI(LOG_TAG, "Ota client conn_handle: %d, subscribed: %s", desc->conn_handle, subValue ? "true" : "false");
    if (!subValue && m_pOta->m_recvLen < m_pOta->m_fileLen &&
        NimBLEAddress(m_pOta->m_client.peer_id_addr) == NimBLEAddress(desc->peer_id_addr)) { // Disconnected, abort
        m_pOta->abortUpdate();
    }
}

void NimBLEOta::NimBLEOtaCharacteristicCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
    if (pCharacteristic->getUUID().equals(COMMAND_UUID)) {
        commandOnWrite(pCharacteristic, desc);
    } else if (pCharacteristic->getUUID().equals(RECV_FW_UUID)) {
        firmwareOnWrite(pCharacteristic, desc);
    }
}

NimBLEService* NimBLEOta::start() {
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
