#ifndef A_BOOTLOADER_H
#define A_BOOTLOADER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>  // For memcpy

// Constants and macros
#define PACKET_SIZE 128             // Size of each packet payload in bytes
#define MAX_FIRMWARE_SIZE 65536     // Maximum size of the firmware image (64 KB)
#define MAX_RETRIES 3               // Maximum number of retries for sending a packet
#define MODBUS_MAX_DATA_SIZE 252    // Maximum data size in a MODBUS frame, constrained by protocol

// GPIO pin for Chip Select (CS) line for SPI communication with MCU B
#define PIN_CS_MCU_B 5  // Example GPIO pin number for MCU B's CS line

// Data Structures

// Structure to represent a firmware packet
typedef struct {
    uint32_t sequence_number;   // Sequence number of the packet
    uint8_t data[PACKET_SIZE];  // Data payload of the packet
    uint16_t crc;               // CRC checksum for error detection
} Packet_t;

// Structure to represent the full firmware image
typedef struct {
    uint8_t data[MAX_FIRMWARE_SIZE]; // Buffer holding the full firmware image
    uint32_t size;                   // Size of the firmware image in bytes
    uint16_t checksum;               // Checksum of the entire firmware image
    uint32_t num_packets;            // Total number of packets to be sent
} FirmwareImage_t;

// Enumeration for update status
typedef enum {
    UPDATE_NOT_STARTED,  // Firmware update has not started
    UPDATE_IN_PROGRESS,  // Firmware update is in progress
    UPDATE_COMPLETED,    // Firmware update completed successfully
    UPDATE_TIMEOUT,      // Timeout occurred during the update
    UPDATE_ERROR         // General error during the update
} UpdateStatus_t;

// Structure to manage the transmission process
typedef struct {
    uint32_t current_packet;     // Index of the packet currently being sent
    UpdateStatus_t update_status; // Status of the firmware update process
    uint32_t retries;            // Number of retries for the current packet
    uint32_t max_retries;        // Maximum number of retries allowed
} TransmissionControl_t;

// Enumeration for SPI communication states
typedef enum {
    SPI_IDLE,             // SPI is idle, not currently transmitting
    SPI_HANDSHAKE,        // Performing handshake with MCU B
    SPI_TRANSMITTING,     // Transmitting packets to MCU B
    SPI_WAITING_ACK,      // Waiting for acknowledgment from MCU B
    SPI_COMPLETE,         // Transmission completed successfully
    SPI_ERROR             // Error occurred during SPI communication
} SPICommState_t;

// Enumeration for MODBUS communication states
typedef enum {
    MODBUS_IDLE,              // MODBUS is idle, not currently receiving
    MODBUS_RECEIVING_PACKET,  // Receiving packets from RS485
    MODBUS_VALIDATE_PACKET,   // Validating received packet
    MODBUS_COMPLETE,          // Reception completed successfully
    MODBUS_ERROR              // Error occurred during MODBUS communication
} ModbusCommState_t;

// MODBUS Frame structure
typedef struct {
    uint8_t address;                   // Slave address (1 byte)
    uint8_t function_code;             // Function code (1 byte)
    uint8_t data[MODBUS_MAX_DATA_SIZE]; // Data field (variable length)
    uint16_t crc;                      // CRC for error-checking (2 bytes)
    uint16_t length;                   // Length of the data field
} ModbusFrame_t;

// Function Prototypes

// System initialization
void System_Init(void);

// Bootloader task
void Bootloader_Task(void);

// Firmware reception via MODBUS/RS485
void Firmware_Reception_Task(void);

// Firmware transmission via SPI to MCU B
void Firmware_Transmission_Task(void);

// Self-firmware update for MCU A
void Self_Firmware_Update_Task(void);

// MODBUS frame preparation and transmission
void Prepare_Modbus_Frame(ModbusFrame_t *frame, uint8_t address, uint8_t function_code, Packet_t *packet);
void Transmit_Modbus_Frame(ModbusFrame_t *frame);

// SPI packet preparation and transmission
void Prepare_SPI_Packet(Packet_t *packet, FirmwareImage_t *firmware, uint32_t packet_number);
void Transmit_SPI_Packet(Packet_t *packet);

// CS/SS line control
void SPI_Select_Slave(void);
void SPI_Deselect_Slave(void);

// Utility functions
bool Modbus_Receive_Packet(Packet_t *packet);
bool Modbus_Validate_Packet(Packet_t *packet);
void Modbus_Acknowledge_Packet(void);
bool Initiate_SPI_Handshake(void);
bool SPI_Handshake_Completed(void);
bool SPI_Ack_Received(void);
bool Timeout_Detected(void);
void Log_Update_Status(UpdateStatus_t status);
void Handle_Error(UpdateStatus_t status);
void Cleanup_Resources(void);

#endif




