#include "b_bootloader.h"

// Initialize necessary data structures
Packet_t receivedPacket;
Metadata_t firmwareMetadata;
FirmwareBuffer_t firmwareBuffer;
UpdateStatus_t updateStatus;
BootloaderState_t bootloaderState = BOOTLOADER_WAIT_HANDSHAKE;

// Initialize system (SPI, GPIOs, and other peripherals)
void System_Init() {
    Initialize_SPI();
    Initialize_GPIO();
    Initialize_Flash();
    Initialize_Watchdog();
    firmwareBuffer.size = 0;
}

// Handle incoming SPI communication in bootloader mode
void Bootloader_Task() {
    while (true) {
        switch (bootloaderState) {

            case BOOTLOADER_WAIT_HANDSHAKE:
                if (SPI_Handshake_Received()) {
                    bootloaderState = BOOTLOADER_RECEIVE_PACKET;
                    Reset_Watchdog();
                } else if (Timeout_Detected()) {
                    updateStatus = UPDATE_TIMEOUT;
                    bootloaderState = BOOTLOADER_ERROR;
                }
                break;

            case BOOTLOADER_RECEIVE_PACKET:
                if (SPI_Receive_Packet(&receivedPacket)) {
                    if (Validate_CRC(receivedPacket)) {
                        Add_Packet_To_Buffer(&firmwareBuffer, &receivedPacket);
                        if (All_Packets_Received()) {
                            bootloaderState = BOOTLOADER_VALIDATE_METADATA;
                        } else {
                            Reset_Watchdog();
                        }
                    } else {
                        updateStatus = UPDATE_ERROR;
                        bootloaderState = BOOTLOADER_ERROR;
                    }
                } else if (Timeout_Detected()) {
                    updateStatus = UPDATE_TIMEOUT;
                    bootloaderState = BOOTLOADER_ERROR;
                }
                break;

            case BOOTLOADER_VALIDATE_METADATA:
                if (Extract_Metadata(&firmwareBuffer, &firmwareMetadata)) {
                    if (Validate_Metadata(&firmwareMetadata)) {
                        bootloaderState = BOOTLOADER_PROGRAM_FLASH;
                    } else {
                        updateStatus = UPDATE_INVALID_METADATA;
                        bootloaderState = BOOTLOADER_ERROR;
                    }
                } else {
                    updateStatus = UPDATE_ERROR;
                    bootloaderState = BOOTLOADER_ERROR;
                }
                break;

            case BOOTLOADER_PROGRAM_FLASH:
                if (Program_Flash(&firmwareBuffer)) {
                    updateStatus = UPDATE_SUCCESS;
                    bootloaderState = BOOTLOADER_RUN_APPLICATION;
                } else {
                    updateStatus = UPDATE_ERROR;
                    bootloaderState = BOOTLOADER_ERROR;
                }
                break;

            case BOOTLOADER_ERROR:
                Log_Update_Status(updateStatus);
                Cleanup_Resources();
                bootloaderState = BOOTLOADER_RUN_APPLICATION;
                break;

            case BOOTLOADER_RUN_APPLICATION:
                if (updateStatus == UPDATE_SUCCESS) {
                    Reset_MCU();
                } else {
                    Run_Existing_Application();
                }
                return;

            default:
                bootloaderState = BOOTLOADER_ERROR;
                break;
        }
    }
}

// SPI Handshake check function
bool SPI_Handshake_Received() {
    // Code to check if the handshake signal from MCU A is received
    return Check_SPI_Handshake_Signal();
}

// Timeout detection function
bool Timeout_Detected() {
    // Check if the operation exceeded the allowed time
    return Check_Timeout();
}

// CRC validation function
bool Validate_CRC(Packet_t packet) {
    // Calculate and compare the CRC of the received packet
    return (Calculate_CRC(packet.data, PACKET_SIZE) == packet.crc);
}

// Add packet to the firmware buffer
void Add_Packet_To_Buffer(FirmwareBuffer_t *buffer, Packet_t *packet) {
    memcpy(&buffer->data[buffer->size], packet->data, PACKET_SIZE);
    buffer->size += PACKET_SIZE;
}

// Check if all firmware packets are received
bool All_Packets_Received() {
    // Determine if the entire firmware has been received
    return (firmwareBuffer.size >= firmwareMetadata.firmware_size);
}

// Extract metadata from the firmware buffer
bool Extract_Metadata(FirmwareBuffer_t *buffer, Metadata_t *metadata) {
    // Code to extract firmware metadata from the buffer
    return Parse_Metadata(buffer->data, metadata);
}

// Validate metadata
bool Validate_Metadata(Metadata_t *metadata) {
    // Check firmware version, size, and checksum
    return (metadata->firmware_version > CURRENT_VERSION &&
            metadata->firmware_size == firmwareBuffer.size &&
            Validate_Checksum(firmwareBuffer.data, firmwareBuffer.size, metadata->checksum));
}

// Program flash memory with the firmware data
bool Program_Flash(FirmwareBuffer_t *buffer) {
    // Erase necessary flash sectors
    if (!Erase_Flash()) {
        return false;
    }
    // Write buffer data to flash memory
    return Write_Flash(buffer->data, buffer->size);
}

// Log the update status
void Log_Update_Status(UpdateStatus_t status) {
    // Store the status in non-volatile memory or transmit to the host
    Store_Status(status);
}

// Cleanup resources before exiting bootloader mode
void Cleanup_Resources() {
    // Deinitialize peripherals, clear buffers, etc.
    Deinitialize_SPI();
    Deinitialize_Watchdog();
}

// Run the existing or new application after the update
void Run_Existing_Application() {
    // Jump to the application code
    Jump_To_Application();
}

void Reset_MCU() {
    // Reset MCU to start the new firmware
    Perform_System_Reset();
}


