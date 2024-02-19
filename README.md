# OBD2_STM32F103C6_ESP
OBD2 Using STM32F103C6 and ESP8266 WiFi interface with a mobile app to control the OBD2 device to request and send data from any vehicle.

# STM32 OBD2 Library

An STM32 OBD2 library for reading and writing data for almost any vehicle through the CAN bus with many features.

## Main Features

- Ability to read PIDs with more than 100 supported PIDs.
- Configuration parameters can be adjusted through the `OBD2_CFG.h` file.
- Ability to read and clear Diagnostic Trouble Codes (DTCs).
- Read Vehicle Identification Number (VIN).
- Read Electronic Control Unit (ECU) Name.
- Read main dashboard information such as speed, temperature, engine load, and more.
- Send and receive frames.

## Services

- 01 Show current data
- 02 Show freeze frame data
- 03 Show stored Diagnostic Trouble Codes
- 04 Clear Diagnostic Trouble Codes and stored values
- 05 Test results, oxygen sensor monitoring (non-CAN only)
- 06 Test results, other component/system monitoring (Test results, oxygen sensor monitoring for CAN only)
- 07 Show pending Diagnostic Trouble Codes (detected during current or last driving cycle)
- 08 Control operation of on-board component/system
- 09 Request vehicle information
- 0A Permanent Diagnostic Trouble Codes (DTCs) (Cleared DTCs)

## Protocols Used

- ISO-TP (ISO 15765-2 Protocol)

## APIs

```c
void OBD2_Init(void);
void OBD2_RequestData(uint8_t OBD2Mode, uint8_t PID);
void OBD2_ReadVIN(uint8_t* Name);
void OBD2_ReadECUName(uint8_t* Name);
void OBD2_ReadDTCs(uint8_t* DTCs);
void OBD2_SendDebugMessage(char *format, ...);

uint8_t OBD2_ClearStoredDTC(void);
uint8_t OBD2_ReadData(uint8_t OBD2Mode, uint8_t PID, uint8_t* Data, uint8_t Size);
int32_t OBD2_FetchData(uint8_t PID);
HAL_StatusTypeDef OBD2_GetDebugMessage(uint8_t *Msg, uint8_t Size);
```

## Compatibility

This library has been successfully tested on a Ford EcoSport 2017 before being published.

## References

- [OBD-II PIDs - Wikipedia](https://en.wikipedia.org/wiki/OBD-II_PIDs)

Feel free to modify and enhance it according to your preferences!

