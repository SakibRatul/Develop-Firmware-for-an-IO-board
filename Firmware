#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h> 

#define NUM_DIO_PINS 8
#define NUM_AI_CHANNELS 2
#define NUM_AO_CHANNELS 2
#define DEFAULT_UART_BAUD 9600
#define DEFAULT_UART_DATA_BITS 8
#define DEFAULT_UART_PARITY 0
#define DEFAULT_UART_STOP_BITS 1
#define MAX_BLINK_FREQUENCY 1000
#define MIN_BLINK_FREQUENCY 1
#define EEPROM_SIZE 256 
#define MAX_CMD_LENGTH 16

// Command Identifiers
#define COMMAND_SET_PIN_MODE 0x01
#define COMMAND_SET_PIN_OUTPUT 0x02
#define COMMAND_SET_ANALOG_OUTPUT 0x03
#define COMMAND_GET_STATUS 0x04
#define COMMAND_SET_BLINK_FREQ 0x05
#define COMMAND_SET_PRIMARY_UART 0x06
#define COMMAND_SET_SECONDARY_UART 0x07

// Error Codes
typedef enum {
    ERROR_NONE = 0,
    ERROR_INVALID_PIN = 1,
    ERROR_INVALID_MODE = 2,
    ERROR_INVALID_VALUE = 3,
    ERROR_UART_CONFIG = 4,
    ERROR_UART_TRANSMIT = 5,
    ERROR_EEPROM = 6,
    ERROR_STATE = 7,
    ERROR_UNKNOWN_COMMAND = 8,
    ERROR_ADC = 9,
    ERROR_COMMAND_LENGTH = 10,
    ERROR_EEPROM_CRC = 11,
} ErrorCode;

// UART Configuration Structure
typedef struct {
    uint32_t baudRate;
    uint8_t dataBits;
    uint8_t parity;
    uint8_t stopBits;
} UARTConfig;

// Global Variables
static uint32_t systemTick = 0;
static uint8_t dioStates[NUM_DIO_PINS] = {0}; // Track output states for output mode
static uint8_t dioModes[NUM_DIO_PINS] = {0}; // Track pin modes
static uint8_t blinkState[NUM_DIO_PINS] = {0};
static uint16_t blinkFrequencies[NUM_DIO_PINS] = {1}; // Frequency in Hz
static uint16_t aiValues[NUM_AI_CHANNELS] = {0}; // Analog input values
static uint16_t aoValues[NUM_AO_CHANNELS] = {0}; // Analog Output values
static UARTConfig primaryUART = {DEFAULT_UART_BAUD, DEFAULT_UART_DATA_BITS, DEFAULT_UART_PARITY, DEFAULT_UART_STOP_BITS};
static UARTConfig secondaryUART = {DEFAULT_UART_BAUD, DEFAULT_UART_DATA_BITS, DEFAULT_UART_PARITY, DEFAULT_UART_STOP_BITS};

// System State Enum
enum SystemState {
    STATE_IDLE = 0,
    STATE_INPUT = 1,
    STATE_BLINKING = 2,
    STATE_OUTPUT = 3,
    STATE_ERROR = 4,
    STATE_CONFIGURING
};

// Current system state
static enum SystemState currentState = STATE_IDLE;

// Simulated EEPROM
static uint8_t eeprom[EEPROM_SIZE] = {0};
static uint16_t eepromWriteAddress = 0; // address tracker

// Function Prototypes
uint32_t calculateCRC32(const uint8_t *data, size_t length);
float mapToVoltage(uint16_t value);
void stateMachineProcess();
void validateAndRecoverConfig();
void handleUARTCommand(uint8_t *command, uint16_t length);
void setDIOPinMode(uint8_t pin, uint8_t mode);
void setAnalogOutput(uint8_t channel, uint16_t value);
void reportSystemStatus();
void uartTransmit(UARTConfig *uart, const char *message);
void uartTransmitError(UARTConfig *uart, const char *message, ErrorCode code);
void configureUART(UARTConfig *uart, uint32_t baudRate, uint8_t dataBits, uint8_t parity, uint8_t stopBits);
uint16_t readAnalogInput(uint8_t channel);
void loadConfigurationsFromEEPROM();
void saveConfigurationsToEEPROM();
void simulateUARTError(UARTConfig *uart);
void sendIOStatus();
uint16_t readADC(uint8_t channel); // Placeholder for real ADC read function
void uartReceive(UARTConfig *uart, uint8_t *buffer, uint16_t maxLength);


// CRC32 Implementation
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
        }
    }
    return ~crc;
}

// Function Definitions
float mapToVoltage(uint16_t value) {
    return (value / 1023.0) * 5.0; // Map 0–1023 to 0–5 V
}

void stateMachineProcess() {
    static uint32_t lastTick[NUM_DIO_PINS] = {0};
    systemTick++;

    // Get Analog Input reads from ADC using the readADC method
    for (int i = 0; i < NUM_AI_CHANNELS; i++) {
        aiValues[i] = readADC(i); // Call the readADC placeholder
        if (aiValues[i] == 0xFFFF) {
            uartTransmitError(&primaryUART, "Error: ADC read failure for channel %d\n", ERROR_ADC);
            currentState = STATE_ERROR;
        }
    }

    for (int pin = 0; pin < NUM_DIO_PINS; pin++) {
        if (dioModes[pin] == STATE_BLINKING) {
            uint32_t blinkPeriod = 1000 / blinkFrequencies[pin];
            if ((systemTick - lastTick[pin]) >= blinkPeriod) {
                lastTick[pin] = systemTick;
                dioStates[pin] = !dioStates[pin];
                printf("Pin %d toggled to state %d at %d Hz\n", pin, dioStates[pin], blinkFrequencies[pin]);
            }
        } else if (dioModes[pin] == STATE_INPUT) {
            // Simulate reading input
            // Do something with the input here like saving or logging.
            printf("Pin %d is in input mode\n", pin);
        } else if (dioModes[pin] == STATE_OUTPUT) {
            printf("Pin %d is in output mode with state %d\n", pin, dioStates[pin]);
        }
    }

    // Check for error state condition based on time for testing
    if (systemTick % 5000 == 0 && currentState != STATE_ERROR) {
        currentState = STATE_ERROR;
        uartTransmit(&secondaryUART, "Error: Transitioning to error state. System will now reset if this was a real device.\n");
    }
    // check to see if an error state was cleared if the system state is ERROR
    if (currentState == STATE_ERROR && systemTick % 500 == 0) {
        currentState = STATE_IDLE; // transition out of error state if it has been there for a while
        uartTransmit(&secondaryUART, "Error Cleared: Transitioning to idle state.\n");
    }

    if (systemTick % 1000 == 0 && currentState != STATE_CONFIGURING) {
        sendIOStatus();
    }
}

void validateAndRecoverConfig() {
    bool configInvalid = false;
    if (primaryUART.baudRate == 0 || primaryUART.dataBits == 0 || primaryUART.stopBits == 0) {
        configInvalid = true;
    }
    if (secondaryUART.baudRate == 0 || secondaryUART.dataBits == 0 || secondaryUART.stopBits == 0) {
        configInvalid = true;
    }

    if (configInvalid) {
        uartTransmitError(&secondaryUART, "Error: Invalid UART configuration. Reverting to defaults.\n", ERROR_UART_CONFIG);
        configureUART(&primaryUART, DEFAULT_UART_BAUD, DEFAULT_UART_DATA_BITS, DEFAULT_UART_PARITY, DEFAULT_UART_STOP_BITS);
        configureUART(&secondaryUART, DEFAULT_UART_BAUD, DEFAULT_UART_DATA_BITS, DEFAULT_UART_PARITY, DEFAULT_UART_STOP_BITS);
        saveConfigurationsToEEPROM(); // save the default values.
    }
}

void handleUARTCommand(uint8_t *command, uint16_t length) {
    if (currentState == STATE_ERROR) {
        uartTransmitError(&primaryUART, "System in Error State. Command Ignored\n", ERROR_STATE);
        return;
    }

    if (length < 1) {
        uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
        return;
    }

    uint8_t commandID = command[0];
    switch (commandID) {
        case COMMAND_SET_PIN_MODE: {
            if (length != 3) {
                uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
                return;
            }
            uint8_t pin = command[1];
            uint8_t mode = command[2];
            setDIOPinMode(pin, mode);
             uartTransmit(&primaryUART, "Pin mode updated.\n");
            break;
        }
        case COMMAND_SET_PIN_OUTPUT: {
            if (length != 3) {
                uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
                return;
            }
            uint8_t pin = command[1];
            uint8_t value = command[2];
            if (pin < NUM_DIO_PINS) {
                if (value == 0 || value == 1) {
                    dioStates[pin] = value;
                    printf("Pin %d set to output mode with state %d.\n", pin, dioStates[pin]);
                     uartTransmit(&primaryUART, "Pin output updated.\n");
                } else {
                    uartTransmitError(&primaryUART, "Error: Invalid pin state for output mode.\n", ERROR_INVALID_MODE);
                }
            } else {
                uartTransmitError(&primaryUART, "Error: Invalid pin number.\n", ERROR_INVALID_PIN);
            }
            break;
        }
        case COMMAND_SET_ANALOG_OUTPUT: {
            if (length != 4) {
                uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
                return;
            }
            uint8_t channel = command[1];
            uint16_t value = (command[2] << 8) | command[3];
            if (value > 1023) {
                uartTransmitError(&primaryUART, "Error: Invalid analog output value.\n", ERROR_INVALID_VALUE);
            } else {
                setAnalogOutput(channel, value);
                 uartTransmit(&primaryUART, "Analog output updated.\n");
            }

            break;
        }
        case COMMAND_GET_STATUS: {
            reportSystemStatus();
            break;
        }
        case COMMAND_SET_BLINK_FREQ: {
            if (length != 4) {
                uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
                return;
            }
            uint8_t pin = command[1];
            uint16_t frequency = (command[2] << 8) | command[3];
            if (pin < NUM_DIO_PINS) {
                if (frequency >= MIN_BLINK_FREQUENCY && frequency <= MAX_BLINK_FREQUENCY) {
                    blinkFrequencies[pin] = frequency;
                    uartTransmit(&primaryUART, "Blink frequency updated.\n");
                } else {
                    uartTransmitError(&primaryUART, "Error: Invalid blink frequency.\n", ERROR_INVALID_VALUE);
                }
            } else {
                uartTransmitError(&primaryUART, "Error: Invalid Pin Number.\n", ERROR_INVALID_PIN);
            }
            break;
        }
        case COMMAND_SET_PRIMARY_UART: {
            if (length != 8) {
                uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
                return;
            }
            currentState = STATE_CONFIGURING;
            uint32_t baud = (command[1] << 24) | (command[2] << 16) | (command[3] << 8) | command[4];
            uint8_t dataBits = command[5];
            uint8_t parity = command[6];
            uint8_t stopBits = command[7];

            configureUART(&primaryUART, baud, dataBits, parity, stopBits);
            saveConfigurationsToEEPROM();
            currentState = STATE_IDLE;
            uartTransmit(&primaryUART, "Primary UART Configured.\n");
            break;
        }
        case COMMAND_SET_SECONDARY_UART: {
            if (length != 8) {
                uartTransmitError(&primaryUART, "Error: Invalid Command Length\n", ERROR_COMMAND_LENGTH);
                return;
            }
             currentState = STATE_CONFIGURING;
            uint32_t baud = (command[1] << 24) | (command[2] << 16) | (command[3] << 8) | command[4];
            uint8_t dataBits = command[5];
            uint8_t parity = command[6];
            uint8_t stopBits = command[7];

            configureUART(&secondaryUART, baud, dataBits, parity, stopBits);
            saveConfigurationsToEEPROM();
            currentState = STATE_IDLE;
           uartTransmit(&primaryUART, "Secondary UART Configured.\n");
            break;
        }
        default: {
            uartTransmitError(&primaryUART, "Unknown command.\n", ERROR_UNKNOWN_COMMAND);
        }
    }
}


void setDIOPinMode(uint8_t pin, uint8_t mode) {
    if (pin < NUM_DIO_PINS) {
        if (mode != STATE_OUTPUT && mode != STATE_BLINKING && mode != STATE_INPUT) {
            uartTransmitError(&primaryUART, "Error: Invalid pin mode.\n", ERROR_INVALID_MODE);
            return;
        }
        dioModes[pin] = mode; //Set pin mode
        if (mode == STATE_OUTPUT) {
            printf("Pin %d set to output mode.\n", pin);
            blinkState[pin] = 0; //Disable blinking
        } else if (mode == STATE_BLINKING) {
            printf("Pin %d set to blink mode.\n", pin);
        } else if (mode == STATE_INPUT) {
            printf("Pin %d set to input mode.\n", pin);
            blinkState[pin] = 0; //Disable blinking
        }
    } else {
        uartTransmitError(&primaryUART, "Error: Invalid pin number.\n", ERROR_INVALID_PIN);
    }
}


void setAnalogOutput(uint8_t channel, uint16_t value) {
    if (channel < NUM_AO_CHANNELS) {
        if (value > 1023) {
            uartTransmitError(&primaryUART, "Error: Invalid analog output value.\n", ERROR_INVALID_VALUE);
            return;
        }
        aoValues[channel] = value;
        float voltage = mapToVoltage(value);
        printf("Analog Output Channel %d set to %.2f V\n", channel, voltage);
    } else {
        uartTransmitError(&primaryUART, "Error: Invalid analog output channel.\n", ERROR_INVALID_PIN);
    }
}

uint16_t readAnalogInput(uint8_t channel) {
    if (channel < NUM_AI_CHANNELS) {
        return aiValues[channel];
    } else {
        uartTransmitError(&primaryUART, "Error: Invalid analog input channel.\n", ERROR_INVALID_PIN);
        return 0xFFFF; // Return an error value
    }
}

void reportSystemStatus() {
    printf("System Status: Tick=%d, Current State=%d\n", systemTick, currentState);
    printf("Primary UART Baud=%d, DataBits=%d, Parity=%d, StopBits=%d\n", primaryUART.baudRate, primaryUART.dataBits, primaryUART.parity, primaryUART.stopBits);
    printf("Secondary UART Baud=%d, DataBits=%d, Parity=%d, StopBits=%d\n", secondaryUART.baudRate, secondaryUART.dataBits, secondaryUART.parity, secondaryUART.stopBits);
    for (int i = 0; i < NUM_DIO_PINS; i++) {
        printf("Pin %d: Mode=%d, Freq=%d\n", i, dioModes[i], blinkFrequencies[i]);
    }
    for (int i = 0; i < NUM_AI_CHANNELS; i++) {
        float voltage = mapToVoltage(aiValues[i]);
        printf("AI Channel %d: Value=%d, Voltage=%.2fV\n", i, aiValues[i], voltage);
    }
    for (int i = 0; i < NUM_AO_CHANNELS; i++) {
        float voltage = mapToVoltage(aoValues[i]);
        printf("AO Channel %d: Value=%d, Voltage=%.2fV\n", i, aoValues[i], voltage);
    }
}

void uartTransmit(UARTConfig *uart, const char *message) {
    // Simulate UART transmission with error checking
    if (message == NULL) {
        uartTransmitError(&primaryUART, "Error: Null message.\n", ERROR_UART_TRANSMIT);
        return;
    }
    printf("UART[%d]: %s", uart->baudRate, message);
}

void uartTransmitError(UARTConfig *uart, const char *message, ErrorCode code) {
    // Simulate UART transmission error
    if (message == NULL) {
        uartTransmit(&secondaryUART, "Error: Null message.\n");
        return;
    }
    printf("UART[%d] Error %d: %s", uart->baudRate, code, message);
}

void configureUART(UARTConfig *uart, uint32_t baudRate, uint8_t dataBits, uint8_t parity, uint8_t stopBits) {
    uart->baudRate = baudRate;
    uart->dataBits = dataBits;
    uart->parity = parity;
    uart->stopBits = stopBits;
    printf("UART configured: Baud=%d, DataBits=%d, Parity=%d, StopBits=%d\n", baudRate, dataBits, parity, stopBits);
}

void simulateUARTError(UARTConfig *uart) {
    // Simulate a framing error or data corruption
    uartTransmitError(uart, "Simulated UART error.\n", ERROR_UART_TRANSMIT);
}


// Simulate saving configurations to non-volatile memory (EEPROM)
void saveConfigurationsToEEPROM() {
  eepromWriteAddress = 0;
  
    // Calculate the data length
    size_t dataLength = sizeof(UARTConfig) * 2 + NUM_DIO_PINS * 2;
    uint8_t data[dataLength];
    size_t offset = 0;

    // Copy primary UART config
    memcpy(data + offset, &primaryUART, sizeof(UARTConfig));
    offset += sizeof(UARTConfig);

    // Copy secondary UART config
    memcpy(data + offset, &secondaryUART, sizeof(UARTConfig));
    offset += sizeof(UARTConfig);

    // Copy DIO pin modes
    memcpy(data + offset, dioModes, NUM_DIO_PINS);
    offset += NUM_DIO_PINS;

    // Copy blink frequencies
    memcpy(data + offset, blinkFrequencies, NUM_DIO_PINS);
    offset += NUM_DIO_PINS;

    // Calculate CRC32 checksum
    uint32_t crc = calculateCRC32(data, dataLength);

    // Store CRC32 at the start of EEPROM
    memcpy(eeprom + eepromWriteAddress, &crc, sizeof(uint32_t));
    eepromWriteAddress += sizeof(uint32_t);

    // Save the data to EEPROM
    memcpy(eeprom + eepromWriteAddress, data, dataLength);
    eepromWriteAddress += dataLength;

    printf("Configurations saved to EEPROM.\n");
}

// Simulate loading configurations from non-volatile memory (EEPROM)
void loadConfigurationsFromEEPROM() {
    eepromWriteAddress = 0;
    uint32_t storedCRC;
    size_t dataLength = sizeof(UARTConfig) * 2 + NUM_DIO_PINS * 2;
     uint8_t data[dataLength];
     size_t offset = 0;

    // Load the stored CRC32
    memcpy(&storedCRC, eeprom + eepromWriteAddress, sizeof(uint32_t));
    eepromWriteAddress += sizeof(uint32_t);

    // Load the data from EEPROM
     memcpy(data, eeprom + eepromWriteAddress, dataLength);
     eepromWriteAddress += dataLength;

    // Calculate the CRC32 checksum
    uint32_t calculatedCRC = calculateCRC32(data, dataLength);

     // Verify the CRC
    if (storedCRC != calculatedCRC) {
        uartTransmitError(&secondaryUART, "Error: EEPROM data is corrupt.\n", ERROR_EEPROM_CRC);
          // Revert to defaults and re-save

           configureUART(&primaryUART, DEFAULT_UART_BAUD, DEFAULT_UART_DATA_BITS, DEFAULT_UART_PARITY, DEFAULT_UART_STOP_BITS);
          configureUART(&secondaryUART, DEFAULT_UART_BAUD, DEFAULT_UART_DATA_BITS, DEFAULT_UART_PARITY, DEFAULT_UART_STOP_BITS);
           memset(dioModes, 0, NUM_DIO_PINS);
          memset(blinkFrequencies, 1, NUM_DIO_PINS);
           saveConfigurationsToEEPROM();
        return;
    }

    // Load primary UART config
     memcpy(&primaryUART, data + offset, sizeof(UARTConfig));
    offset += sizeof(UARTConfig);

    // Load secondary UART config
    memcpy(&secondaryUART, data + offset, sizeof(UARTConfig));
    offset += sizeof(UARTConfig);

    // Load DIO pin modes
     memcpy(dioModes, data+offset, NUM_DIO_PINS);
    offset += NUM_DIO_PINS;

     // Load blink frequencies
    memcpy(blinkFrequencies, data+offset, NUM_DIO_PINS);
    offset += NUM_DIO_PINS;

    printf("Configurations loaded from EEPROM.\n");
}

void sendIOStatus() {
    char status[256]; // buffer for status message
    snprintf(status, sizeof(status), "STATUS: Tick=%d, State=%d, AI0=%d, AI1=%d, AO0=%d, AO1=%d,", systemTick, currentState, aiValues[0], aiValues[1], aoValues[0], aoValues[1]);
    for (int i = 0; i < NUM_DIO_PINS; i++) {
        snprintf(status + strlen(status), sizeof(status) - strlen(status), "DIO%d=%d,%d,", i, dioModes[i], dioStates[i]);
    }
    status[strlen(status) - 1] = '\0';// removes trailing comma
    uartTransmit(&primaryUART, status);
    uartTransmit(&primaryUART, "\n");
}

// Placeholder for real ADC read function
uint16_t readADC(uint8_t channel) {
    // Simulate a varying value for AI channels
    static uint16_t simulatedValues[NUM_AI_CHANNELS] = {200, 800};
     static int16_t noiseOffset[NUM_AI_CHANNELS] = {0}; // Add a noise offset

    if (channel < NUM_AI_CHANNELS) {
        noiseOffset[channel] = (noiseOffset[channel] + (rand() % 10 - 5));
        return (simulatedValues[channel] + noiseOffset[channel]) & 0x3FF; // Ensure value stays within bounds 0-1023
    } else {
        return 0xFFFF; // Return an error value
    }
}

// UART Receive (Simplified example - needs actual UART driver logic)
void uartReceive(UARTConfig *uart, uint8_t *buffer, uint16_t maxLength) {
    //Simulate a receive of data to a buffer
    if (rand() % 5 == 0) { // Simulate a random error
        uartTransmitError(uart, "Error: Simulation UART Error\n", ERROR_UART_TRANSMIT);
        buffer[0] = 0xFF;
        return;
    }
    int length = 3 + (rand() % (maxLength-3)); // max length is set when calling the function

    for (int i = 0; i < length; i++) {
        buffer[i] = rand() % 0xFF;
    }

    handleUARTCommand(buffer, length); // Pass the received data to the handler
}



int main() {
    // Seed the random number generator
    srand(time(NULL));

    // Simulate some EEPROM data for testing
    primaryUART.baudRate = 115200;
    primaryUART.dataBits = 8;
    primaryUART.parity = 0;
    primaryUART.stopBits = 1;

    secondaryUART.baudRate = 9600;
    secondaryUART.dataBits = 8;
    secondaryUART.parity = 0;
    secondaryUART.stopBits = 1;
    saveConfigurationsToEEPROM();


    loadConfigurationsFromEEPROM(); // Load configurations from simulated EEPROM
    validateAndRecoverConfig(); // Validate the loaded configurations

    uint8_t rxBuffer[MAX_CMD_LENGTH];


    // Set pin 1 to blink at 10Hz
    uint8_t setPin1Blink[] = {COMMAND_SET_PIN_MODE, 1, STATE_BLINKING};
    handleUARTCommand(setPin1Blink, sizeof(setPin1Blink));

    uint8_t setPin1Freq[] = {COMMAND_SET_BLINK_FREQ, 1, 0, 10}; //10 Hz
    handleUARTCommand(setPin1Freq, sizeof(setPin1Freq));

    // Set pin 2 to blink at 20Hz
    uint8_t setPin2Blink[] = {COMMAND_SET_PIN_MODE, 2, STATE_BLINKING};
    handleUARTCommand(setPin2Blink, sizeof(setPin2Blink));

    uint8_t setPin2Freq[] = {COMMAND_SET_BLINK_FREQ, 2, 0, 20}; // 20Hz
    handleUARTCommand(setPin2Freq, sizeof(setPin2Freq));
    // Set pin 3 to input
    uint8_t setPin3Input[] = {COMMAND_SET_PIN_MODE, 3, STATE_INPUT};
    handleUARTCommand(setPin3Input, sizeof(setPin3Input));

    // Set pin 4 to output high
    uint8_t setPin4Output[] = {COMMAND_SET_PIN_MODE, 4, STATE_OUTPUT};
    handleUARTCommand(setPin4Output, sizeof(setPin4Output));
    uint8_t setPin4High[] = {COMMAND_SET_PIN_OUTPUT, 4, 1};
    handleUARTCommand(setPin4High, sizeof(setPin4High));


    // Set Analog output 0 to 512
    uint8_t setAO0[] = {COMMAND_SET_ANALOG_OUTPUT, 0, 0x02, 0x00};
    handleUARTCommand(setAO0, sizeof(setAO0));

    // Set Analog output 1 to 100
    uint8_t setAO1[] = {COMMAND_SET_ANALOG_OUTPUT, 1, 0x00, 0x64};
    handleUARTCommand(setAO1, sizeof(setAO1));

    // Simulate state machine
    for (int i = 0; i < 1000; i++) {
        stateMachineProcess();
        if (rand() % 20 == 0) { // simulate a random UART error
           simulateUARTError(&primaryUART);
        }
         if(i % 100 == 0){
             uartReceive(&primaryUART, rxBuffer, MAX_CMD_LENGTH);
         }
    }

    // Get the Status command
     uint8_t getStatus[] = {COMMAND_GET_STATUS};
    handleUARTCommand(getStatus, sizeof(getStatus));

    // Set the primary UART command
    uint8_t setPrimaryUART[] = {COMMAND_SET_PRIMARY_UART, 0x00, 0x01, 0xC2, 0x00, 8, 0, 1}; // 115200, 8, 0, 1
    handleUARTCommand(setPrimaryUART, sizeof(setPrimaryUART));

    // Set the secondary UART command
    uint8_t setSecondaryUART[] = {COMMAND_SET_SECONDARY_UART, 0x00, 0x00, 0x4B, 0x00, 8, 1, 1}; // 19200, 8, 1, 1
    handleUARTCommand(setSecondaryUART, sizeof(setSecondaryUART));

    // Get the Status command
    handleUARTCommand(getStatus, sizeof(getStatus));
    return 0;
}
