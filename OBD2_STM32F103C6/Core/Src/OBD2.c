
/*******************************************************************************
 *  [FILE NAME]   :      <OBD2.c>                                              *
 *  [AUTHOR]      :      <David S. Alexander>                                  *
 *  [DATE CREATED]:      <Feb 16, 2024>                                        *
 *  [Description} :      <Source file for OBD2 driver>                         *
 *******************************************************************************/

/*******************************************************************************
 *                                  Includes                                   *
 *******************************************************************************/
#include "OBD2.h"

const char* PID_Names[] =
{
  "",
  "Monitor Status Since DTCs Cleared",
  "Freez DTC",
  "Fuel System Status",
  "Engine Load",
  "Engine Coolant Temperature",
  "Short Term Fuel Trim 1",
  "Long Term Fuel Trim 1",
  "Short Term Fuel Trim 2",
  "Long Term Fuel Trim 2",
  "Fuel Pressure",
  "Intake Manifold Pressure",
  "Engine RPM",
  "Vehicle Speed",
  "Timing Advance",
  "Intake Air Temperature",
  "MAF Air Flow Rate",
  "Throttle Position",
  "Commanded Secondary Air Status",
  "Oxygen Sensors Present (2 BANKS)",
  "Oxygen Sensor 1 Short Term Fuel Trim",
  "Oxygen Sensor 2 Short Term Fuel Trim",
  "Oxygen Sensor 3 Short Term Fuel Trim",
  "Oxygen Sensor 4 Short Term Fuel Trim",
  "Oxygen Sensor 5 Short Term Fuel Trim",
  "Oxygen Sensor 6 Short Term Fuel Trim",
  "Oxygen Sensor 7 Short Term Fuel Trim",
  "Oxygen Sensor 8 Short Term Fuel Trim",
  "OBD Standards",
  "Oxygen Sensors Present (4 BANKS)",
  "Auxiliary Input Status",
  "Run Time Since Engine Start",
  "PIDs Supported (21-40)",
  "Distance Traveled With MIL",
  "Fuel Rail Pressure",
  "Fuel Rail Gauge Pressure",
  "Oxygen Sensor 1 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 2 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 3 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 4 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 5 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 6 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 7 Fuel Air Equivalent Ratio",
  "Oxygen Sensor 8 Fuel Air Equivalent Ratio",
  "Commanded EGR",
  "EGR Error",
  "Commanded Evaporative Purge",
  "Fuel Tank Level Input",
  "Warm Ups Since Codes Cleared",
  "Distance Traveled Since Codes Clear",
  "System Vapor Pressure",
  "Absolute Barometric Pressure",
  "", "", "", "", "", "", "", "",
  "Catalyst Temperature Bank 1 Sensor 1",
  "Catalyst Temperature Bank 2 Sensor 1",
  "Catalyst Temperature Bank 1 Sensor 2",
  "Catalyst Temperature Bank 2 Sensor 2",
  "Monitor Status Current Drive Cycle",
  "Control Module Voltage",
  "Absolute Load Value",
  "",
  "Relative Throttle Position",
  "Outside Temperature",
  "Absolute Throttle Position B",
  "Absolute Throttle Position C",
  "Absolute Throttle Position D",
  "Absolute Throttle Position E",
  "Absolute Throttle Position F",
  "Commanded Throttle Actuator",
  "Time Run With MIL On",
  "Time Since Trouble Codes Cleared",
  "",
  "Fuel Type",
  "Ethanol Fuel Percentage",
  "", "", "", "", "", "",
  "Fuel Rail Absolute Pressure",
  "Relative Accelerator Pedal Position",
  "Hybrid Battery Pack Remaining Life",
  "Engine Oil Temperature",
  "Fuel Injection Timing",
  "Engine Fuel Rate",
};

CAN_TxHeaderTypeDef TxHeaderCAN;
CAN_RxHeaderTypeDef RxHeaderCAN;
CAN_FilterTypeDef   CANFilterConfig;
uint32_t TxMailbox;
uint32_t RxMailbox;
uint8_t TxDataCAN[TX_CAN_BUFFER];
uint8_t RxDataCAN[RX_CAN_BUFFER];
volatile uint8_t isPacketReceived = PACKET_NOT_RECEIVED;

/**
 * @brief Decodes four bytes of OBD2 data into a 32-bit unsigned integer.
 *
 * @param ByteA First byte of OBD2 data.
 * @param ByteB Second byte of OBD2 data.
 * @param ByteC Third byte of OBD2 data.
 * @param ByteD Fourth byte of OBD2 data.
 * @return uint32_t Decoded 32-bit unsigned integer.
 */
static uint32_t OBD2_DataDecode(uint8_t ByteA, uint8_t ByteB, uint8_t ByteC, uint8_t ByteD)
{
    return ((uint32_t)ByteA << 24 | (uint32_t)ByteB << 16 | (uint32_t)ByteC << 8 | (uint32_t)ByteD);
}

/**
 * @brief Sends the next CAN frame for OBD2 communication.
 *
 * @param Message The OBD2 message to be sent.
 */
static void OBD2_SendNextFrame(uint8_t Message)
{
    /* Set CAN header parameters */
	TxHeaderCAN.StdId = RequestMessagesID1;       /* Send ID */
	TxHeaderCAN.ExtId = 0x00;                     /* No extended ID */
	TxHeaderCAN.RTR = CAN_RTR_DATA;               /* Data frame */
	TxHeaderCAN.IDE = CAN_ID_STD;                 /* Standard ID */
	TxHeaderCAN.DLC = TX_CAN_BUFFER;              /* Length of data frame */
	TxHeaderCAN.TransmitGlobalTime = DISABLE;

    /* Fill data bytes */
	TxDataCAN[0] = Message;
	TxDataCAN[1] = 0x00;
	TxDataCAN[2] = 0x00;
	TxDataCAN[3] = 0x00;
	TxDataCAN[4] = 0x00;
	TxDataCAN[5] = 0x00;
	TxDataCAN[6] = 0x00;
	TxDataCAN[7] = 0x00;

	/* Transmit CAN message */
	HAL_CAN_AddTxMessage(&OBD2CAN, &TxHeaderCAN, TxDataCAN, &TxMailbox);
}

/**
 * @brief Initializes OBD2 communication.
 *
 * This function activates CAN receive interrupt, configures CAN filter bank,
 * and starts CAN communication.
 */
void OBD2_Init(void)
{
    /* Enable CAN receive interrupt */
    HAL_CAN_ActivateNotification(&OBD2CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
    TxMailbox = 0;
    RxMailbox = 0;

    /* Configure filter bank 0 for ID 0x7E8 & 0x7EA */
    CANFilterConfig.FilterBank           = 0;
    CANFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;
    CANFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CANFilterConfig.FilterIdHigh         = ECMResponseMessagesID << 5;
    CANFilterConfig.FilterIdLow          = 0;
                                           /* SAE Standard IDs */
    CANFilterConfig.FilterMaskIdHigh     = ((ECMResponseMessagesID | 0x7EA) << 5);
    CANFilterConfig.FilterMaskIdLow      = 0;
    CANFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    CANFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    CANFilterConfig.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&OBD2CAN, &CANFilterConfig);
    HAL_CAN_Start(&OBD2CAN);
}

/**
 * @brief Requests OBD2 data using the specified mode and PID.
 *
 * @param OBD2Mode The OBD2 mode to use for the request.
 * @param PID The PID (Parameter ID) of the data being requested.
 *
 * This function prepares and sends a CAN message requesting OBD2 data
 * with the specified mode and PID.
 */
void OBD2_RequestData(uint8_t OBD2Mode, uint8_t PID)
{
    TxHeaderCAN.StdId = RequestMessagesID;        /* Request ID */
    TxHeaderCAN.ExtId = 0x00;                     /* No extended ID */
    TxHeaderCAN.RTR = CAN_RTR_DATA;               /* Data frame */
    TxHeaderCAN.IDE = CAN_ID_STD;                 /* Standard ID */
    TxHeaderCAN.DLC = TX_CAN_BUFFER;              /* Length of data frame */
    TxHeaderCAN.TransmitGlobalTime = DISABLE;

    TxDataCAN[0] = 0x02;                          /* Number of additional data bytes */
    TxDataCAN[1] = OBD2Mode;                      /* Request Live Data */
    TxDataCAN[2] = PID;                           /* OBD PID that we are requesting */
    TxDataCAN[3] = 0x00;
    TxDataCAN[4] = 0x00;
    TxDataCAN[5] = 0x00;
    TxDataCAN[6] = 0x00;
    TxDataCAN[7] = 0x00;

    /* Transmit CAN message */
    HAL_CAN_AddTxMessage(&OBD2CAN, &TxHeaderCAN, TxDataCAN, &TxMailbox);
}

/**
 * @brief Sends a debug message through the UART.
 *
 * @param format The format string for the message.
 * @param ... Additional arguments to be formatted according to the format string.
 *
 * This function allows sending debug messages with variable arguments
 * using printf-style formatting to the UART for debugging purposes.
 */
void OBD2_SendDebugMessage(char *format, ...)
{
    char Messsage[DEBUG_MESSAGE_BUFFER] = {0};
    /* Holds the information needed by va_start, va_arg, va_end */
    va_list args;
    /* Enables access to the variable arguments */
    va_start(args, format);
    /* Write formatted data from variable argument list to string */
    vsprintf(Messsage, format, args);
    /* Transmit the formatted data through the defined UART */
    HAL_UART_Transmit(&SERIAL_DEBUG, (uint8_t *)Messsage, sizeof(Messsage), HAL_MAX_DELAY);
    /* Performs cleanup for an ap object initialized by a call to va_start */
    va_end(args);
}

/**
 * @brief Clears stored Diagnostic Trouble Codes (DTC) in the vehicle's engine control module.
 *
 * @return uint8_t Status of the operation:
 *                - CLEARE_SUCCESS if the clearing operation is successful.
 *                - An error code indicating the failure reason if the operation fails.
 *
 * This function sends a CAN message to clear any stored DTC (Diagnostic Trouble Codes)
 * in the engine control module of the vehicle. It waits for a response indicating
 * the success or failure of the operation and returns the status accordingly.
 */
uint8_t OBD2_ClearStoredDTC(void)
{
    TxHeaderCAN.StdId = RequestMessagesID1;           /* Functional Broadcast Request ID */
    TxHeaderCAN.ExtId = 0x00;                         /* No extended ID */
    TxHeaderCAN.RTR = CAN_RTR_DATA;                   /* Data frame */
    TxHeaderCAN.IDE = CAN_ID_STD;                     /* Standard ID */
    TxHeaderCAN.DLC = TX_CAN_BUFFER;                  /* Length of data frame */
    TxHeaderCAN.TransmitGlobalTime = DISABLE;

    TxDataCAN[0] = 0x14;                              /* Number of additional data bytes */
    TxDataCAN[1] = ClearStoredEmissionsRelatedData;   /* Clear any stored DTC */
    TxDataCAN[2] = 0x00;
    TxDataCAN[3] = 0x00;
    TxDataCAN[4] = 0x00;
    TxDataCAN[5] = 0x00;
    TxDataCAN[6] = 0x00;
    TxDataCAN[7] = 0x00;

    /* Transmit CAN message */
    HAL_CAN_AddTxMessage(&OBD2CAN, &TxHeaderCAN, TxDataCAN, &TxMailbox);
    HAL_Delay(50);

    /* Check if the DTCs cleared */
    if (isPacketReceived == PACKET_RECEIVED && RxDataCAN[1] == ACK_MESSAGE_ID)
    {
        isPacketReceived = PACKET_NOT_RECEIVED;
        return CLEARE_SUCCESS;
    }
    else
    {
    	/* Error clearing the DTCs, return the clear error code */
        return RxDataCAN[1];
    }
}

/**
 * @brief Reads OBD2 data using the specified mode and PID with ISO-TP 15765-2 Protocol.
 *
 * @param OBD2Mode The OBD2 mode to use for the request.
 * @param PID The PID (Parameter ID) of the data being requested.
 * @param Data Pointer to a buffer where the received data will be stored.
 * @param Size Size of the buffer pointed to by Data.
 * @return uint8_t The size of the data received, or 0 if an error occurs.
 *
 * This function requests OBD2 data using the specified mode and PID,
 * reads the response from the CAN bus, and stores it in the provided buffer.
 * It handles single-frame and multi-frame responses, ensuring all data is properly received.
 * The function returns the size of the received data or 0 if an error occurs.
 */
uint8_t OBD2_ReadData(uint8_t OBD2Mode, uint8_t PID, uint8_t* Data, uint8_t Size)
{
    /* Request OBD2 data */
    OBD2_RequestData(OBD2Mode, PID);
    HAL_Delay(80); /* Wait for response */

    uint8_t BytesCounter = 0;
    uint8_t DataSize = 0;

    /* Check if packet received */
    if (isPacketReceived == PACKET_RECEIVED)
    {
        /* Handle single-frame response */
        if (RxDataCAN[0] == SF_ID)
        {
            DataSize = RxDataCAN[1]; /* Get data size */
            if (DataSize > NO_BYTES_RECEIVED)
            {
                /* Copy data to buffer */
                *((uint32_t*)&Data[0]) = *((uint32_t*)&RxDataCAN[4]);

                /* Handle multi-frame response if data size exceeds single-frame maximum */
                if (DataSize > SF_MAX_SIZE)
                {
                    uint8_t Length = SF_MAX_SIZE;
                    uint8_t TimeOutVal = 0;
                    isPacketReceived = PACKET_NOT_RECEIVED;

                    /* Loop through frames */
                    for (uint8_t FramesCounts = 0; (Length < DataSize && Length < Size); FramesCounts++)
                    {
                        /* Send flow control frame */
                        OBD2_SendNextFrame(FC_ID);

                        /* Wait for next frame */
                        while (isPacketReceived == PACKET_NOT_RECEIVED || (RxDataCAN[0] != (FF_ID + FramesCounts)))
                        {
                            HAL_Delay(1);
                            if (TimeOutVal++ >= 50)
                            {
                                return 0; /* Timeout, return 0 */
                            }
                        }

                        /* Copy received data to buffer */
                        for (BytesCounter = 1; BytesCounter < RX_CAN_BUFFER /*&& (Length < DataSize && Length < Size) */; BytesCounter++)
                        {
                            Data[Length++] = RxDataCAN[BytesCounter];
                        }
                        isPacketReceived = PACKET_NOT_RECEIVED;
                        TimeOutVal = 0;
                    }
                }
            }
        }
        else /* Handle frame response with data size less than 5 bytes */
        {
            *((uint32_t*)&Data[0]) = *((uint32_t*)&RxDataCAN[3]);
            DataSize = 4;
        }
    }
    return DataSize; /* Return data size */
}

/**
 * @brief Reads the Vehicle Identification Number (VIN) using OBD2.
 *
 * @param Name Pointer to a buffer where the VIN will be stored.
 *
 * This function reads the Vehicle Identification Number (VIN) from the vehicle's
 * engine control module using OBD2 and stores it in the provided buffer.
 */
void OBD2_ReadVIN(uint8_t* Name)
{
    /* Read VIN data using OBD2 */
    OBD2_ReadData(RequestVehicleInformation, PID_VIN, Name, VIN_SIZE);
}

/**
 * @brief Reads the Engine Control Unit (ECU) name using OBD2.
 *
 * @param Name Pointer to a buffer where the ECU name will be stored.
 *
 * This function reads the Engine Control Unit (ECU) name from the vehicle's
 * engine control module using OBD2 and stores it in the provided buffer.
 */
void OBD2_ReadECUName(uint8_t* Name)
{
    /* Read ECU name data using OBD2 */
    OBD2_ReadData(RequestVehicleInformation, PID_ECUName, Name, ECU_NAME_SIZE);
}

/**
 * @brief Reads Diagnostic Trouble Codes (DTCs) from the vehicle's engine control module using OBD2.
 *
 * @param DTCs Pointer to a buffer where the DTCs will be stored.
 *
 * This function reads Diagnostic Trouble Codes (DTCs) from the vehicle's engine control module
 * using OBD2 and stores them in the provided buffer. It processes the received data and formats
 * the DTCs before storing them in the buffer.
 */
void OBD2_ReadDTCs(uint8_t* DTCs)
{
    uint8_t DCTs_Buffer[DTCS_MAX_SIZE];
    /* Get the data length */
    uint8_t DataLength = OBD2_ReadData(RequestStoredTroubleCodes, 0x00, DCTs_Buffer, DTCS_MAX_SIZE);
    /* Check the number of DTCs */
    if (DataLength >= SINGLE_DTC_LENGTH)
    {
        uint8_t BytesCounter = 0;
        uint8_t DTCsCounter = 0;
        uint8_t FrameError = E_OK;
        for (; BytesCounter < DataLength; BytesCounter += SINGLE_DTC_LENGTH)
        {
        	/* Decode the DTC ID */
            switch (DCTs_Buffer[BytesCounter] & 0xF0)
            {
                case OBD2_DTC_Powertrain:
                    DTCs[DTCsCounter++] = 'P';
                    break;
                case OBD2_DTC_Chassis:
                    DTCs[DTCsCounter++] = 'C';
                    break;
                case OBD2_DTC_Body:
                    DTCs[DTCsCounter++] = 'B';
                    break;
                case OBD2_DTC_Network:
                    DTCs[DTCsCounter++] = 'U';
                    break;
                default:
                	/* Data corrupted ! */
                    FrameError = E_NOT_OK;
            }
            if (FrameError != E_NOT_OK)
            {
            	/* Decode the code in the array of characters */
                DTCs[DTCsCounter++] = '0';
                DTCs[DTCsCounter++] = (DCTs_Buffer[BytesCounter]     & 0x0F) + '0';
                DTCs[DTCsCounter++] = (DCTs_Buffer[BytesCounter + 1] & 0xF0) + '0';
                DTCs[DTCsCounter++] = (DCTs_Buffer[BytesCounter + 1] & 0x0F) + '0';
                DTCs[DTCsCounter++] = '\0';
            }
            else
            {
                memcpy(DTCs, "Frame Error !", sizeof("Frame Error !"));
                break;
            }
        }
    }
    else
    {
    	/* This response or P0000 indicates that no DTCs stored */
        memcpy(DTCs, "No DTCs Exist", sizeof("No DTCs Exist"));
    }
}

/**
 * @brief Fetches and decodes OBD2 data based on the given PID.
 *
 * @param PID The Parameter ID (PID) for which data is to be fetched.
 * @return int32_t The decoded value of the fetched OBD2 data.
 *
 * This function fetches and decodes OBD2 data based on the given PID (Parameter ID).
 * It processes the raw data received from the CAN bus and returns the decoded value
 * corresponding to the specified PID.
 * These calculations are based on the great documentation of OBD-II
 * you can find here https://en.wikipedia.org/wiki/OBD-II_PIDs
 */
int32_t OBD2_FetchData(uint8_t PID)
{
    switch (PID)
    {
      case PID_EngineRPM:
        return ((RxDataCAN[3] * 255 + RxDataCAN[4]) / 4);

      case PID_MonitorStatusSinceDTCsCleared:
      case PID_FreezDTC:
      case PID_MonitorStatusCurrentDriveCycle:
        return (OBD2_DataDecode(RxDataCAN[3], RxDataCAN[4], RxDataCAN[5], RxDataCAN[6]));

      case PID_FuelSystemStatus:
      case PID_RunTimeSinceEngineStart:
      case PID_DistanceTraveledWithMIL:
      case PID_DistanceTraveledSinceCodesClear:
      case PID_TimeRunWithMILOn:
      case PID_TimeSinceTroubleCodesCleared:
        return (RxDataCAN[3] * 256.0 + RxDataCAN[4]);

      case PID_EngineOilTemp:
      case PID_EngineCoolantTemp:
      case PID_IntakeAirTemp:
      case PID_OutsideTemp:
        return (RxDataCAN[3] - 40);

      case PID_EngineLoad:
      case PID_ThrottlePosition:
      case PID_CommandedEGR:
      case PID_CommandedEvaporativePurge:
      case PID_FuelTankLevelInput:
      case PID_RelativeThrottlePosition:
      case PID_AbsoluteThrottlePositionB:
      case PID_AbsoluteThrottlePositionC:
      case PID_AbsoluteThrottlePositionD:
      case PID_AbsoluteThrottlePositionE:
      case PID_AbsoluteThrottlePositionF:
      case PID_CommandedThrottleActuator:
      case PID_EthanolFuelPercentage:
      case PID_RelativeAcceleratorPedalPosition:
      case PID_HybridBatteryPackRemainingLife:
        return (RxDataCAN[3] / 2.55);

      case PID_CommandedSecondaryAirStatus:
      case PID_OBDStandards:
      case PID_OxygenSensorsPresent_2BANKS:
      case PID_OxygenSensorsPresent_4BANKS:
      case PID_AuxiliaryInputStatus:
      case PID_FuelType:
        return (RxDataCAN[3]);

      case PID_OxygenSensor1ShortTermFuelTrim:
      case PID_OxygenSensor2ShortTermFuelTrim:
      case PID_OxygenSensor3ShortTermFuelTrim:
      case PID_OxygenSensor4ShortTermFuelTrim:
      case PID_OxygenSensor5ShortTermFuelTrim:
      case PID_OxygenSensor6ShortTermFuelTrim:
      case PID_OxygenSensor7ShortTermFuelTrim:
      case PID_OxygenSensor8ShortTermFuelTrim:
        return ((RxDataCAN[4] / 1.28) - 100.0);

      case PID_ShortTermFuelTrim1:
      case PID_LongTermFuelTrim1:
      case PID_ShortTermFuelTrim2:
      case PID_LongTermFuelTrim2:
      case PID_EGRError:
        return ((RxDataCAN[3] / 1.28) - 100.0);

      case PID_FuelPressure:
        return (RxDataCAN[3] * 3.0);

      case PID_IntakeManifoldPressure:
      case PID_VehicleSpeed:
      case PID_WarmUpsSinceCodesCleared:
      case PID_AbsoluteBarometricPressure:
        return (RxDataCAN[3]);

      case PID_TimingAdvance:
        return ((RxDataCAN[3] / 2.0) - 64.0);

      case PID_MAF_AirFlowRate:
        return ((RxDataCAN[3] * 256.0 + RxDataCAN[4]) / 100.0);

      case PID_FuelRailPressure:
        return ((RxDataCAN[3] * 256.0 + RxDataCAN[4]) * 0.079);

      case PID_FuelRailGaugePressure:
      case PID_FuelRailAbsolutePressure:
        return ((RxDataCAN[3] * 256.0 + RxDataCAN[4]) * 10.0);

      case PID_OxygenSensor1FuelAirEquivRatio:
      case PID_OxygenSensor2FuelAirEquivRatio:
      case PID_OxygenSensor3FuelAirEquivRatio:
      case PID_OxygenSensor4FuelAirEquivRatio:
      case PID_OxygenSensor5FuelAirEquivRatio:
      case PID_OxygenSensor6FuelAirEquivRatio:
      case PID_OxygenSensor7FuelAirEquivRatio:
      case PID_OxygenSensor8FuelAirEquivRatio:
        return (((RxDataCAN[3] * 256.0 + RxDataCAN[4]) * 2.0) / 65536.0);

      case PID_SystemVaporPressure:
        return (((int16_t)(RxDataCAN[3] * 256.0 + RxDataCAN[4])) / 4.0);

      case PID_ControlModuleVoltage:
        return ((RxDataCAN[3] * 256.0 + RxDataCAN[4]) / 1000.0);

      case PID_AbsoluteLoadValue:
        return ((RxDataCAN[3] * 256.0 + RxDataCAN[4]) / 2.55);

      case PID_FuelInjectionTiming:
        return (((RxDataCAN[3] * 256.0 + RxDataCAN[4]) / 128.0) - 210.0);

      case PID_EngineFuelRate:
        return ((RxDataCAN[3] * 256.0 + RxDataCAN[4]) / 20.0);

      default:
        /* error, No other message should be received */
        return 0;
    }

  return 0;
}

/**
 * @brief Retrieves a debug message from the debug serial interface.
 *
 * @param Msg Pointer to the buffer where the message will be stored.
 * @param Size Size of the buffer.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 *
 * This function retrieves a debug message from the debug serial interface
 * within a specified timeout period. The received message is stored in the provided buffer.
 */
HAL_StatusTypeDef OBD2_GetDebugMessage(uint8_t *Msg, uint8_t Size)
{
	return HAL_UART_Receive(&SERIAL_DEBUG, Msg, Size, 100);
}

/**
 * @brief Callback function invoked when a message is pending in CAN Rx FIFO0.
 *
 * @param hcan Pointer to a CAN_HandleTypeDef structure that contains
 *              the configuration information for the specified CAN.
 *
 * This function is a callback invoked when a pending message interrupt fires.
 * It retrieves the message from the CAN Rx FIFO 0 and checks the Protocol ID (PID).
 * Upon successful retrieval of the message, it sets the flag isPacketReceived to indicate
 * that a packet has been received.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* When a pending message interrupt fires, get the message and check the PID */
	if (HAL_CAN_GetRxMessage(OBD2CAN, CAN_RX_FIFO0, &RxHeaderCAN, RxDataCAN) == HAL_OK)
	{
		/* Uncomment the line below for debugging purposes */
		/* OBD2_SendDebugMessage("Received CAN message: ID=0x%lx, DLC=%d\n", RxHeaderCAN.StdId, RxHeaderCAN.DLC); */
		isPacketReceived = PACKET_RECEIVED;
	}
}
