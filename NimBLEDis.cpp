// Copyright 2025 Ryan Powell and NimBLEOta contributors
// Sponsored by Theengs https://www.theengs.io
// MIT License

#include "NimBLEDevice.h"
#include "NimBLEDis.h"
#include "NimBLELog.h"

#define BLE_DIS_SERVICE_UUID               (uint16_t)0x180a
#define BLE_DIS_SYSTEM_ID_CHR_UUID         (uint16_t)0x2A23
#define BLE_DIS_MODEL_NUMBER_CHR_UUID      (uint16_t)0x2A24
#define BLE_DIS_SERIAL_NUMBER_CHR_UUID     (uint16_t)0x2A25
#define BLE_DIS_FIRMWARE_REVISION_CHR_UUID (uint16_t)0x2A26
#define BLE_DIS_HARDWARE_REVISION_CHR_UUID (uint16_t)0x2A27
#define BLE_DIS_SOFTWARE_REVISION_CHR_UUID (uint16_t)0x2A28
#define BLE_DIS_MANUFACTURER_NAME_CHR_UUID (uint16_t)0x2A29
#define BLE_DIS_REG_CERT_CHR_UUID          (uint16_t)0x2A2A
#define BLE_DIS_PNP_ID_CHR_UUID            (uint16_t)0x2A50

static const char* LOG_TAG = "NimBLEDis";

bool NimBLEDis::createDisChar(const NimBLEUUID& uuid, const uint8_t* value, uint16_t length) {
    if (!m_pDisService) {
        NIMBLE_LOGE(LOG_TAG, "Device Information Service not initialized");
        return false;
    }

    if (m_pDisService->getCharacteristic(uuid)) {
        NIMBLE_LOGE(LOG_TAG,
                    "%s - Characteristic value already set",
                    NimBLEUUID(BLE_DIS_MODEL_NUMBER_CHR_UUID).toString().c_str());
        return false;
    }

    NimBLECharacteristic* pChar = m_pDisService->createCharacteristic(uuid, NIMBLE_PROPERTY::READ, length);
    if (!pChar) {
        NIMBLE_LOGE(LOG_TAG, "Failed to create characteristic");
        return false;
    }

    pChar->setValue(value, length);
    if (memcmp(pChar->getValue(), value, length) != 0) {
        NIMBLE_LOGE(LOG_TAG, "Failed to set characteristic value");
        return false;
    }

    return true;
}

bool NimBLEDis::setModelNumber(const char* value) {
    if (createDisChar(BLE_DIS_MODEL_NUMBER_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "Model Number set to: %s", value);
        return true;
    }
    return false;
}

bool NimBLEDis::setSerialNumber(const char* value) {
    if (createDisChar(BLE_DIS_SERIAL_NUMBER_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "Serial Number set to: %s", value);
    }
    return false;
}

bool NimBLEDis::setFirmwareRevision(const char* value) {
    if (createDisChar(BLE_DIS_FIRMWARE_REVISION_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "Firmware Revision set to: %s", value);
        return true;
    }
    return false;
}

bool NimBLEDis::setHardwareRevision(const char* value) {
    if (createDisChar(BLE_DIS_HARDWARE_REVISION_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "Hardware Revision set to: %s", value);
        return true;
    }
    return false;
}

bool NimBLEDis::setSoftwareRevision(const char* value) {
    if (createDisChar(BLE_DIS_SOFTWARE_REVISION_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "Software Revision set to: %s", value);
        return true;
    }
    return false;
}

bool NimBLEDis::setManufacturerName(const char* value) {
    if (createDisChar(BLE_DIS_MANUFACTURER_NAME_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "Manufacturer Name set to: %s", value);
        return true;
    }
    return false;
}

bool NimBLEDis::setSystemId(const char* value) {
    if (createDisChar(BLE_DIS_SYSTEM_ID_CHR_UUID, reinterpret_cast<const uint8_t*>(value), strlen(value))) {
        NIMBLE_LOGI(LOG_TAG, "System ID set to: %s", value);
        return true;
    }
    return false;
}

bool NimBLEDis::setPnp(uint8_t src, uint16_t vid, uint16_t pid, uint16_t ver) {
    uint8_t pnp[] = {src,
                     static_cast<uint8_t>(vid & 0xFF),
                     static_cast<uint8_t>((vid >> 8) & 0xFF),
                     static_cast<uint8_t>(pid & 0xFF),
                     static_cast<uint8_t>((pid >> 8) & 0xFF),
                     static_cast<uint8_t>(ver & 0xFF),
                     static_cast<uint8_t>((ver >> 8) & 0xFF)};

    if (createDisChar(BLE_DIS_PNP_ID_CHR_UUID, pnp, sizeof(pnp))) {
        NIMBLE_LOGI(LOG_TAG, "PNP ID set to: %02x:%04x:%04x:%04x", src, vid, pid, ver);
        return true;
    }
    return false;
}

bool NimBLEDis::init() {
    NimBLEServer* pServer = NimBLEDevice::createServer();
    if (!pServer) {
        NIMBLE_LOGE(LOG_TAG, "Failed to get server");
        return false;
    }

    m_pDisService = pServer->createService(BLE_DIS_SERVICE_UUID);
    if (!m_pDisService) {
        NIMBLE_LOGE(LOG_TAG, "Failed to create service");
        return false;
    }

    NIMBLE_LOGD(LOG_TAG, "Device Information Service created");
    return true;
}

bool NimBLEDis::start() {
    if (m_pDisService != nullptr) {
        return m_pDisService->start();
    }

    NIMBLE_LOGE(LOG_TAG, "Device Information Service not initialized");
    return false;
}
