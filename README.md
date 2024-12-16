# NimBLE OTA

`NimBLE OTA` is based on the [Espressif ble ota component](https://components.espressif.com/components/espressif/ble_ota/versions/0.1.12) which will update the firmware of an esp32 device Over The Air via Bluetooth Low Energy.

This creates an OTA Service with the UUID `0x8018` and an optional [Device Information Service.](https://www.bluetooth.com/specifications/specs/device-information-service-1-1/)

## Sponsored by

<a href = "https://www.theengs.io"><img src="extras/logo-Theengs.jpg" height="100"/></a>

## Dependencies

[NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) Version 2.1 or higher. Or [esp-nimble-cpp](https://github.com/h2zero/esp-nimble-cpp) Version 2.0 or higher.

## Supported MCU's

Espressif esp32 devices with Bluetooth capability.

## Using
```
#include <NimBLEOta.h> // include the header in your source file

static NimBLEOta bleOta; // Create an instance of NimBLEOta

void setup() {
    NimBLEDevice::init("NimBLE OTA"); // Initialize NimBLE
    bleOta.start(); // start the service
}
```

If you want to have callbacks to your application to receive information about BLE OTA operations you can pass an optional pointer to your callback class which must be derived from `NimBLEOtaCallbacks`.
```
class OtaCallbacks : public NimBLEOtaCallbacks {
} otaCallbacks;

void setup() {
    NimBLEDevice::init("NimBLE OTA"); // Initialize NimBLE
    bleOta.start(&otaCallbacks); // start the service with callbacks
}
```

Check out the examples for more detail.

### Security

If you want to enable security you should initialize the NimBLE security options before calling `bleOta.start()`, you must enable man in the middle protection and use a passkey like so:
```
// Use passkey authentication
NimBLEDevice::setSecurityAuth(false, true, true);
NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
```

You can now start NimBLEOta with security enabled by pass `true` as the second parameter to `start()`, i.e. `bleOta.start(&otaCallbacks, true);`

## Uploading OTA

For best results use the included [python script.](scripts\nimbleota.py)  
This also works with [BLEOTA_WEBAPP](https://gb88.github.io/BLEOTA/) by @gb88.

## 1. How it works

- `OTA Service`: It is used for OTA upgrade and contains 4 characteristics, as shown in the following table:

|  Characteristics   | UUID  |  Prop   | description  |
|  ----  | ----  |  ----  | ----  |
|  RECV_FW_CHAR | 0x8020 | Write, notify  | Firmware received, send ACK |
|  PROGRESS_BAR_CHAR  | 0x8021 | Read, notify  | Read the progress bar and report the progress bar |
|  COMMAND_CHAR  | 0x8022 | Write, notify  | Send the command and ACK |
|  CUSTOMER_CHAR  | 0x8023 | Write, notify  | User-defined data to send and receive |

## 2. Data transmission

### 2.1 Command package format

|  unit   | Command_ID  |  PayLoad   | CRC16  |
|  ----  | ----  |  ----  | ----  |
|  Byte | Byte: 0 ~ 1 | Byte: 2 ~ 17  | Byte: 18 ~ 19 |

Command_ID:

- 0x0001: Start OTA, Payload bytes(2 to 5), indicates the length of the firmware. Other Payload is set to 0 by default. CRC16 calculates bytes(0 to 17).
- 0x0002: Stop OTA, and the remaining Payload will be set to 0. CRC16 calculates bytes(0 to 17).
- 0x0003: The Payload bytes(2 or 3) is the payload of the Command_ID for which the response will be sent. Payload bytes(4 to 5) is a response to the command. 0x0000 indicates accept, 0x0001 indicates reject. Other payloads are set to 0. CRC16 computes bytes(0 to 17).

### 2.2 Firmware package format

The format of the firmware package sent by the client is as follows:

|  unit   | Sector_Index  |  Packet_Seq   | PayLoad  |
|  ----  | ----  |  ----  | ----  |
|  Byte | Byte: 0 ~ 1 | Byte: 2  | Byte: 3 ~ (MTU_size - 4) |

- Sector_Index：Indicates the number of sectors, sector number increases from 0, cannot jump, must be send 4K data and then start transmit the next sector, otherwise it will immediately send the error ACK for request retransmission.
- Packet_Seq：If Packet_Seq is 0xFF, it indicates that this is the last packet of the sector, and the last 2 bytes of Payload is the CRC16 value of 4K data for the entire sector, the remaining bytes will set to 0x0. Server will check the total length and CRC of the data from the client, reply the correct ACK, and then start receive the next sector of firmware data.

The format of the reply packet is as follows:

|  unit   | Sector_Index  |  ACK_Status   | CRC6  |
|  ----  | ----  |  ----  | ----  |
|  Byte | Byte: 0 ~ 1 | Byte: 2 ~ 3  | Byte: 18 ~ 19 |

ACK_Status:

- 0x0000: Success
- 0x0001: CRC error
- 0x0002: Sector_Index error, bytes(4 ~ 5) indicates the desired Sector_Index
- 0x0003：Payload length error
