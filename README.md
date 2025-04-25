
[Click here to read the PDF](mydoc.pdf) – This document contains a detailed description of the project, including architecture, implementation notes, and usage instructions.

# Case Study: Using the PN532 RFID/NFC Module with STM32 via SPI

## Introduction to RFID and NFC Communication

Radio-Frequency Identification (RFID) enables data transmission via radio waves. Passive tags lack a battery and are powered by the reader's field. NFC, operating at 13.56 MHz, is a subset of RFID and supports two-way communication.

## The PN532 NFC Controller: Overview and Features

The PN532 is a versatile NFC controller supporting modes:
- Reader/Writer
- Card Emulation
- Peer-to-Peer

Supported protocols:
- ISO/IEC 14443 A/B
- MIFARE Classic
- FeliCa
- ISO/IEC 18092 (NFCIP-1)

Communication options:
- SPI
- I²C
- UART

## SPI Communication with PN532

SPI is a full-duplex master-slave protocol. STM32 (master) connects with PN532 (slave) via:
- **SCLK** (Serial Clock)
- **MOSI** (Master Out, Slave In)
- **MISO** (Master In, Slave Out)
- **NSS** (Chip Select)

PN532 uses **SPI Mode 0**, **LSB first**, max frequency **1.2 MHz**.

## Communication Protocol and Frame Structure

1. Send command
2. Poll status register
3. Read ACK
4. Poll again
5. Read response

### Frame Types

- **Command**: `00 00 FF LEN LCS TFI=0xD4 Cmd D1..Dn DCS 00`
- **ACK**: `00 00 FF 00 FF 00`
- **Response**: `00 00 FF LEN LCS TFI=0xD5 Cmd+1 D1..Dn DCS 00`

## Essential Commands

### GetFirmwareVersion (0x02)

```text
Sent:     D4 02
Response: D5 03 32 01 06 07
```

### SAMConfiguration (0x14)

Sets passive mode, timeout, IRQ control.

### InListPassiveTarget (0x4A)

Detects RFID tags. Returns:
- Number of targets
- UID

## STM32 SPI Functions

```c
static inline void SS_LOW(void) {
    HAL_GPIO_WritePin(PN532_SS_GPIO_Port, PN532_SS_Pin, GPIO_PIN_RESET);
}

static inline void SS_HIGH(void) {
    HAL_GPIO_WritePin(PN532_SS_GPIO_Port, PN532_SS_Pin, GPIO_PIN_SET);
}

static uint8_t SPI_read(SPI_HandleTypeDef *hspi) {
    uint8_t RxData[1];
    HAL_SPI_Receive(hspi, RxData, 1, HAL_MAX_DELAY);
    return RxData[0];
}

static void SPI_write(SPI_HandleTypeDef *hspi, uint8_t data) {
    uint8_t TxData[1] = { data };
    HAL_SPI_Transmit(hspi, TxData, 1, HAL_MAX_DELAY);
}
```

## Internal Helper Functions

### isReadyToSend()

Checks PN532's status register.

### waitToBeReady(wait_time)

Waits for readiness within timeout.

### readACK()

Reads and validates ACK frame.

### sendFrame(cmd, len)

Builds and sends PN532 frame over SPI.

### sendCommand(cmd, len)

Sends frame, waits, validates ACK.

### readResponseToCommand(cmd, buffer, len, timeout)

Reads full response from PN532.

## Exported API Functions

### PN532_SPI_Init()

Wakes up the PN532.

### PN532_getFirmwareVersion()

Reads firmware version.

### PN532_SAMConfiguration()

Configures SAM mode and RF interface.

### InListPassiveTarget()

Detects tags and returns UID.

## PN532.h Header File (Key Excerpts)

```c
#define PN532_SS_GPIO_Port GPIOB
#define PN532_SS_Pin GPIO_PIN_4

#define PN532_COMMAND_GETFIRMWAREVERSION (0x02)
#define PN532_COMMAND_SAMCONFIGURATION (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)
```

## Using the PN532 API in main.c

```c
uint8_t uid[7] = {0};
uint8_t uidLength;

int main(void) {
    PN532_SPI_Init();
    while (PN532_getFirmwareVersion() == STATUS_532_ERROR) HAL_Delay(250);

    while (PN532_SAMConfiguration() != STATUS_532_OK) HAL_Delay(100);

    while (1) {
        if (InListPassiveTarget(uid, &uidLength) == STATUS_532_OK) {
            // Process UID
        }
        HAL_Delay(1000);
    }
}
```

---

This document is based on content from the EmbeddedBook-2 PDF and is formatted for clarity and reuse.
