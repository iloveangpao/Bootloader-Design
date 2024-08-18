// bootloader.h
#ifndef B_BOOTLOADER_H
#define B_BOOTLOADER_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_SIZE 256
#define MAX_FIRMWARE_SIZE 2^16
#define VERSION_ADDRESS 0xFFFFFFFF

// Define the packet structure
typedef struct {
    uint32_t sequence_number;
    uint8_t data[PACKET_SIZE];  // Assume PACKET_SIZE is defined elsewhere
    uint16_t crc;
} Packet_t;

// Define the metadata structure
typedef struct {
    uint32_t firmware_version;
    uint32_t firmware_size;  // Size in bytes
    uint16_t checksum;       // Overall checksum of the firmware
} Metadata_t;

// Define the update status enumeration
typedef enum {
    UPDATE_SUCCESS,
    UPDATE_TIMEOUT,
    UPDATE_ERROR,
    UPDATE_INVALID_METADATA
} UpdateStatus_t;

// Define the firmware buffer structure
typedef struct {
    uint8_t data[MAX_FIRMWARE_SIZE];  // Assume MAX_FIRMWARE_SIZE is defined elsewhere
    uint32_t size;                    // Current size of data in the buffer
} FirmwareBuffer_t;

// Define the bootloader state enumeration
typedef enum {
    BOOTLOADER_WAIT_HANDSHAKE,
    BOOTLOADER_RECEIVE_PACKET,
    BOOTLOADER_VALIDATE_METADATA,
    BOOTLOADER_PROGRAM_FLASH,
    BOOTLOADER_ERROR,
    BOOTLOADER_RUN_APPLICATION
} BootloaderState_t;

// Function prototypes (if needed)
void System_Init(void);
void Bootloader_Task(void);
bool SPI_Handshake_Received(void);
bool Timeout_Detected(void);
bool Validate_CRC(Packet_t packet);
void Add_Packet_To_Buffer(FirmwareBuffer_t *buffer, Packet_t *packet);
bool All_Packets_Received(void);
bool Extract_Metadata(FirmwareBuffer_t *buffer, Metadata_t *metadata);
bool Validate_Metadata(Metadata_t *metadata);
bool Program_Flash(FirmwareBuffer_t *buffer);
void Log_Update_Status(UpdateStatus_t status);
void Cleanup_Resources(void);
void Run_Existing_Application(void);
void Reset_MCU(void);

#endif
