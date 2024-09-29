#include "NimBLEDevice.h"
#include "NimBLEOta.h"
#include "esp_ota_ops.h"

#define BLE_OTA_SERVICE_UUID (uint16_t)0x8018
#define RECV_FW_UUID (uint16_t)0x8020
#define OTA_BAR_UUID (uint16_t)0x8021
#define COMMAND_UUID (uint16_t)0x8022
#define CUSTOMER_UUID (uint16_t)0x8023
#define BUF_LENGTH 4098
#define OTA_IDX_NB 4
#define CMD_ACK_LENGTH 20
#define START_OTA 0x0001
#define STOP_OTA 0x0002
#define ACK_OTA 0x0003
#define ACK 0x0000
#define NACK 0x0001
#define SIGN_ERROR 0x0003
#define CRC_ERROR 0x0001
#define INDEX_ERROR 0x0002
#define START_ERROR 0x0005

static uint32_t ota_total_len = 0;
static uint32_t ota_block_size = BUF_LENGTH;
static uint8_t *fw_buf = nullptr;
static bool ota_started = false;
static uint32_t cur_sector = 0;
static uint32_t cur_packet = 0;
static uint32_t fw_buf_offset = 0;
static esp_ota_handle_t out_handle = 0;

static const char *LOG_TAG = "NimBLEOta";

static uint16_t crc16_ccitt(const uint8_t *buf, int len)
{
    uint16_t crc16 = 0;
    int32_t i;

    while (len--) {
        crc16 ^= *buf++ << 8;

        for (i = 0; i < 8; i++) {
            if (crc16 & 0x8000) {
                crc16 = (crc16 << 1) ^ 0x1021;
            } else {
                crc16 = crc16 << 1;
            }
        }
    }

    return crc16;
}

class RecvFwCallbacks : public NimBLECharacteristicCallbacks {
    const esp_partition_t *partition_ptr = nullptr;
    esp_partition_t partition;
    const esp_partition_t *next_partition = nullptr;
    uint32_t recv_len = 0;

    void onWrite(NimBLECharacteristic *pCharacteristic) {
        if (!ota_started) {
            NIMBLE_LOGE(LOG_TAG, "ota not started");
            return;
        }

        const uint8_t *data = pCharacteristic->getValue();
        auto data_len = pCharacteristic->getValue().length();
        uint8_t cmd_ack[CMD_ACK_LENGTH] = {0x03, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00
                                          };
        uint16_t crc16 = data[18] | (data[19] << 8);
        uint32_t write_len = 0;

        if (partition_ptr == NULL) {
            partition_ptr = esp_ota_get_next_update_partition(NULL);
            if (partition_ptr == NULL) {
                NIMBLE_LOGE(LOG_TAG, "boot partition NULL!\r\n");
                goto OTA_ERROR;
            }
            if (partition_ptr->type != ESP_PARTITION_TYPE_APP) {
                NIMBLE_LOGE(LOG_TAG, "esp_current_partition->type != ESP_PARTITION_TYPE_APP\r\n");
                goto OTA_ERROR;
            }


            if (partition_ptr->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
                partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
            } else {
                next_partition = esp_ota_get_next_update_partition(partition_ptr);
                if (next_partition) {
                    partition.subtype = next_partition->subtype;
                } else {
                    partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
                }
            }
            partition.type = ESP_PARTITION_TYPE_APP;

            partition_ptr = (esp_partition_t *)esp_partition_find_first(partition.type, partition.subtype, NULL);
            if (partition_ptr == NULL) {
                NIMBLE_LOGE(LOG_TAG, "partition NULL!\r\n");
                goto OTA_ERROR;
            }

            memcpy(&partition, partition_ptr, sizeof(esp_partition_t));
            if (esp_ota_begin(&partition, OTA_SIZE_UNKNOWN, &out_handle) != ESP_OK) {
                NIMBLE_LOGE(LOG_TAG, "esp_ota_begin failed!\r\n");
                goto OTA_ERROR;
            }
        }

        if ((data[0] + (data[1] * 256)) != cur_sector) {
            // sector error
            if ((data[0] == 0xff) && (data[1] == 0xff)) {
                // last sector
                NIMBLE_LOGD(LOG_TAG, "Last sector");
            } else {
                // sector error
                NIMBLE_LOGE(LOG_TAG, "%s - sector index error, cur: %" PRIu32 ", recv: %d", __func__,
                        cur_sector, (data[0] + (data[1] * 256)));
                cmd_ack[0] = data[0];
                cmd_ack[1] = data[1];
                cmd_ack[2] = 0x02; //sector index error
                cmd_ack[3] = 0x00;
                cmd_ack[4] = cur_sector & 0xff;
                cmd_ack[5] = (cur_sector & 0xff00) >> 8;
                crc16 = crc16_ccitt(cmd_ack, 18);
                cmd_ack[18] = crc16 & 0xff;
                cmd_ack[19] = (crc16 & 0xff00) >> 8;
            }
        }

        if (data[2] != cur_packet) { // packet seq error
            if (data[2] == 0xff) { // last packet
                NIMBLE_LOGD(LOG_TAG, "last packet");
                goto write_ota_data;
            } else { // packet seq error
                NIMBLE_LOGE(LOG_TAG, "%s - packet index error, cur: %" PRIu32 ", recv: %d", __func__,
                        cur_packet, data[2]);
            }
        }

    write_ota_data:
        memcpy(fw_buf + fw_buf_offset, data + 3, data_len - 3);
        fw_buf_offset += data_len - 3;

        NIMBLE_LOGD(LOG_TAG, "DEBUG: Sector:%" PRIu32 ", total length:%" PRIu32 ", length:%d", cur_sector,
                fw_buf_offset, data_len - 3);

        if (data[2] == 0xff) {
            cur_packet = 0;
            cur_sector++;
            NIMBLE_LOGD(LOG_TAG, "DEBUG: recv %" PRIu32 " sector", cur_sector);
            goto sector_end;
        } else {
            NIMBLE_LOGD(LOG_TAG, "DEBUG: wait next packet");
            cur_packet++;
        }
        return;

    sector_end:
        write_len = std::min<uint32_t>(4096, fw_buf_offset);
        if (esp_ota_write(out_handle, (const void *)fw_buf, write_len) != ESP_OK) {
            NIMBLE_LOGE(LOG_TAG, "esp_ota_write failed!\r\n");
            goto OTA_ERROR;
        }

        recv_len += write_len;
        if (recv_len >= ota_total_len) {
            if (esp_ota_end(out_handle) != ESP_OK) {
                NIMBLE_LOGE(LOG_TAG, "esp_ota_end failed!\r\n");
                goto OTA_ERROR;
            }

            if (esp_ota_set_boot_partition(&partition) != ESP_OK) {
                NIMBLE_LOGE(LOG_TAG, "esp_ota_set_boot_partition failed!\r\n");
                goto OTA_ERROR;
            }

            esp_restart();
        }

        fw_buf_offset = 0;
        memset(fw_buf, 0x0, ota_block_size);

        cmd_ack[0] = data[0];
        cmd_ack[1] = data[1];
        cmd_ack[2] = 0x00; //success
        cmd_ack[3] = 0x00;
        crc16 = crc16_ccitt(cmd_ack, 18);
        cmd_ack[18] = crc16 & 0xff;
        cmd_ack[19] = (crc16 & 0xff00) >> 8;
        pCharacteristic->setValue(cmd_ack, CMD_ACK_LENGTH);
        pCharacteristic->indicate();
        return;

    OTA_ERROR:
        NIMBLE_LOGE(LOG_TAG, "OTA failed");
        esp_ota_abort(out_handle);
    }
};

class OtaBarCallbacks : public NimBLECharacteristicCallbacks {

};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic * pCharacteristic) {
        uint8_t cmd_ack[CMD_ACK_LENGTH] = {0x03, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00};
        uint16_t crc16 = 0;

        if (pCharacteristic->getValue().length() == 20) {
            const uint8_t *data = pCharacteristic->getValue();
            uint16_t cmd = data[0] | (data[1] << 8);
            crc16 = data[18] | (data[19] << 8);

            if (crc16_ccitt(data, 18) != crc16 || cmd != START_OTA || cmd != STOP_OTA) {
                NIMBLE_LOGE(LOG_TAG, "command %s error", cmd == START_OTA ? "CRC" : cmd == STOP_OTA ? "CRC" : "invalid");
            } else if (cmd == START_OTA)
            {
                ota_total_len = (data[2]) + (data[3] * 256) + (data[4] * 256 * 256) + (data[5] * 256 * 256 * 256);
                NIMBLE_LOGI(LOG_TAG, "recv ota start cmd, fw_length = %" PRIu32 "", ota_total_len);

                fw_buf = (uint8_t *)malloc(ota_block_size * sizeof(uint8_t));
                if (fw_buf == nullptr) {
                    NIMBLE_LOGE(LOG_TAG, "%s -  malloc fail", __func__);
                } else {
                    memset(fw_buf, 0x0, ota_block_size);
                }

                cmd_ack[2] = 0x01;
                cmd_ack[3] = 0x00;
                crc16 = crc16_ccitt(cmd_ack, 18);
                cmd_ack[18] = crc16 & 0xff;
                cmd_ack[19] = (crc16 & 0xff00) >> 8;
                ota_started = true;
                pCharacteristic->setValue(cmd_ack, CMD_ACK_LENGTH);
                pCharacteristic->indicate();
                return;
            } else if (cmd == STOP_OTA) {
                NIMBLE_LOGI(LOG_TAG, "recv ota stop cmd");
                if (fw_buf != nullptr) {
                    free(fw_buf);
                    fw_buf = nullptr;
                }
                cmd_ack[2] = 0x02;
                cmd_ack[3] = 0x00;
                crc16 = crc16_ccitt(cmd_ack, 18);
                cmd_ack[18] = crc16 & 0xff;
                cmd_ack[19] = (crc16 & 0xff00) >> 8;
                ota_started = false;
                esp_ota_abort(out_handle);
                pCharacteristic->setValue(cmd_ack, CMD_ACK_LENGTH);
                pCharacteristic->indicate();
                return;
            }
        } else {
            NIMBLE_LOGE(LOG_TAG, "command length error");
        }

        // command error - send reject ack
        cmd_ack[2] = 0x00;
        cmd_ack[3] = 0x01;
        cmd_ack[4] = 0x01;
        cmd_ack[5] = 0x00;
        crc16 = crc16_ccitt(cmd_ack, 18);
        cmd_ack[18] = crc16 & 0xff;
        cmd_ack[19] = (crc16 & 0xff00) >> 8;
        pCharacteristic->setValue(cmd_ack, CMD_ACK_LENGTH);
        pCharacteristic->indicate();
    }
};

class CustomerCallbacks : public NimBLECharacteristicCallbacks {

};

NimBLEServer *NimBLEOta::createServer()
{
    NimBLEServer *pServer = NimBLEDevice::createServer();
    NimBLEService *pService = pServer->createService(BLE_OTA_SERVICE_UUID);

    /* Receive Firmware Characteristic */
    NimBLECharacteristic *pRecvFwCharacteristic = pService->createCharacteristic(RECV_FW_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pRecvFwCharacteristic->setCallbacks(new RecvFwCallbacks());

    /* OTA Bar Characteristic */
    NimBLECharacteristic *pOtaBarCharacteristic = pService->createCharacteristic(OTA_BAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pOtaBarCharacteristic->setCallbacks(new OtaBarCallbacks());

    /* Command Characteristic */
    NimBLECharacteristic *pCommandCharacteristic = pService->createCharacteristic(COMMAND_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pCommandCharacteristic->setCallbacks(new CommandCallbacks());

    /* Customer Characteristic */
    NimBLECharacteristic *pCustomerCharacteristic = pService->createCharacteristic(CUSTOMER_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pCustomerCharacteristic->setCallbacks(new CustomerCallbacks());

    pService->start();

    return pServer;
}

NimBLEUUID NimBLEOta::getServiceUUID()
{
    return NimBLEUUID{BLE_OTA_SERVICE_UUID};
}