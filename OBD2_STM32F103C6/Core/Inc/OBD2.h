
/*******************************************************************************
 *  [FILE NAME]   :      <OBD2.h>                                              *
 *  [AUTHOR]      :      <David S. Alexander>                                  *
 *  [DATE CREATED]:      <Feb 16, 2024>                                        *
 *  [Description} :      <Header file for OBD2 driver>                         *
 *******************************************************************************/

#ifndef INC_OBD2_H_
#define INC_OBD2_H_

/*******************************************************************************
 *                                  Includes                                   *
 *******************************************************************************/
#include <OBD2_CFG.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*******************************************************************************
 *                             Macro Declarations                              *
 *******************************************************************************/

#define SAE_STANDARD                         0x02
#define VEHICLE_SPECIFIC                     0x03

#define RX_CAN_BUFFER                        0x08
#define TX_CAN_BUFFER                        0x08

#define VIN_SIZE                             0x11
#define ECU_NAME_SIZE                        0x14
#define DTCS_MAX_SIZE                        0x32
#define SINGLE_DTC_LENGTH                    0x02

#define PACKET_RECEIVED                      0x01
#define PACKET_NOT_RECEIVED                  0x00

#define CLEARE_SUCCESS                       0x01
#define CLEARE_FAILED                        0x00

#define E_OK                                 0x01
#define E_NOT_OK                             0x00

#define NO_BYTES_RECEIVED                    0x01

#define SF_ID                                0x10
#define FF_ID                                0x21
#define FC_ID                                0x30
#define SF_MAX_SIZE                          0x04
#define ACK_MESSAGE_ID                       0x65

/* Service 01 - Show current data & Service 02 - Show freeze frame data PIDs */
#define PID_MonitorStatusSinceDTCsCleared    0x01
#define PID_FreezDTC                         0x02
#define PID_FuelSystemStatus                 0x03
#define PID_EngineLoad                       0x04
#define PID_EngineCoolantTemp                0x05
#define PID_ShortTermFuelTrim1               0x06
#define PID_LongTermFuelTrim1                0x07
#define PID_ShortTermFuelTrim2               0x08
#define PID_LongTermFuelTrim2                0x09
#define PID_FuelPressure                     0x0A
#define PID_IntakeManifoldPressure           0x0B
#define PID_EngineRPM                        0x0C
#define PID_VehicleSpeed                     0x0D
#define PID_TimingAdvance                    0x0E
#define PID_IntakeAirTemp                    0x0F
#define PID_MAF_AirFlowRate                  0x10
#define PID_ThrottlePosition                 0x11
#define PID_CommandedSecondaryAirStatus      0x12
#define PID_OxygenSensorsPresent_2BANKS      0x13
#define PID_OxygenSensor1ShortTermFuelTrim   0x14
#define PID_OxygenSensor2ShortTermFuelTrim   0x15
#define PID_OxygenSensor3ShortTermFuelTrim   0x16
#define PID_OxygenSensor4ShortTermFuelTrim   0x17
#define PID_OxygenSensor5ShortTermFuelTrim   0x18
#define PID_OxygenSensor6ShortTermFuelTrim   0x19
#define PID_OxygenSensor7ShortTermFuelTrim   0x1A
#define PID_OxygenSensor8ShortTermFuelTrim   0x1B
#define PID_OBDStandards                     0x1C
#define PID_OxygenSensorsPresent_4BANKS      0x1D
#define PID_AuxiliaryInputStatus             0x1E
#define PID_RunTimeSinceEngineStart          0x1F
#define PID_PIDsSupported_21_40              0x20
#define PID_DistanceTraveledWithMIL          0x21
#define PID_FuelRailPressure                 0x22
#define PID_FuelRailGaugePressure            0x23
#define PID_OxygenSensor1FuelAirEquivRatio   0x24
#define PID_OxygenSensor2FuelAirEquivRatio   0x25
#define PID_OxygenSensor3FuelAirEquivRatio   0x26
#define PID_OxygenSensor4FuelAirEquivRatio   0x27
#define PID_OxygenSensor5FuelAirEquivRatio   0x28
#define PID_OxygenSensor6FuelAirEquivRatio   0x29
#define PID_OxygenSensor7FuelAirEquivRatio   0x2A
#define PID_OxygenSensor8FuelAirEquivRatio   0x2B
#define PID_CommandedEGR                     0x2C
#define PID_EGRError                         0x2D
#define PID_CommandedEvaporativePurge        0x2E
#define PID_FuelTankLevelInput               0x2F
#define PID_WarmUpsSinceCodesCleared         0x30
#define PID_DistanceTraveledSinceCodesClear  0x31
#define PID_SystemVaporPressure              0x32
#define PID_AbsoluteBarometricPressure       0x33

#define PID_CatalystTemperatureBank1Sensor1  0x3C
#define PID_CatalystTemperatureBank2Sensor1  0x3D
#define PID_CatalystTemperatureBank1Sensor2  0x3E
#define PID_CatalystTemperatureBank2Sensor2  0x3F
#define PID_MonitorStatusCurrentDriveCycle   0x41
#define PID_ControlModuleVoltage             0x42
#define PID_AbsoluteLoadValue                0x43

#define PID_RelativeThrottlePosition         0x45
#define PID_OutsideTemp                      0x46
#define PID_AbsoluteThrottlePositionB        0x47
#define PID_AbsoluteThrottlePositionC        0x48
#define PID_AbsoluteThrottlePositionD        0x49
#define PID_AbsoluteThrottlePositionE        0x4A
#define PID_AbsoluteThrottlePositionF        0x4B
#define PID_CommandedThrottleActuator        0x4C
#define PID_TimeRunWithMILOn                 0x4D
#define PID_TimeSinceTroubleCodesCleared     0x4E

#define PID_FuelType                         0x51
#define PID_EthanolFuelPercentage            0x52

#define PID_FuelRailAbsolutePressure         0x59
#define PID_RelativeAcceleratorPedalPosition 0x5A
#define PID_HybridBatteryPackRemainingLife   0x5B
#define PID_EngineOilTemp                    0x5C
#define PID_FuelInjectionTiming              0x5D
#define PID_EngineFuelRate                   0x5E

/* Service 09 - Request vehicle information PIDs */
#define PID_VINMessageCountinPID02           0x01
#define PID_VIN                              0x02
#define PID_CalibrationIDMsgCountforPID04    0x03
#define PID_CalibrationID                    0x04
#define PID_CVNMsgCount                      0x05
#define PID_CVN                              0x06
#define PID_InUsePerformanceTrackMsgcount    0x07
#define PID_InUsePerformanceTrackSparkV      0x08
#define PID_ECUNameMessageCountPID0A         0x09
#define PID_ECUName                          0x0A
#define PID_InUsePerformanceTrackCIgnitionV  0x0B




/*******************************************************************************
 *                            Data Types Declaration                           *
 *******************************************************************************/
typedef enum
{
    RequestLiveData                		  = 0x01,
    RequestFreezeFrame              	  = 0x02,
    RequestStoredTroubleCodes       	  = 0x03,
    ClearStoredEmissionsRelatedData		  = 0x04,
    RequestOxygenSensorData        		  = 0x05,
    RequestMonitoringTestResults  		  = 0x06,
    RequestPendingTroubleCodes    		  = 0x07,
    RequestControlOfOnboardSystem  		  = 0x08,
    RequestVehicleInformation      	      = 0x09,
    RequestPermanentTroubleCodes    	  = 0x0A,
} OBD2_Modes_t;

typedef enum
{
    RequestMessagesID               	  = 0x7DF,
	RequestMessagesID1               	  = 0x7E0,
	ECMResponseMessagesID                 = 0x7E8,
	ResponseMessagesID1               	  = 0x7E9,
	ResponseMessagesID2               	  = 0x7EF,
} OBD2_SAE_IDs_t;

typedef enum
{
    OBD2_DTC_Powertrain                   = 0x0,
	OBD2_DTC_Chassis                      = 0x4,
	OBD2_DTC_Body                         = 0x8,
	OBD2_DTC_Network                      = 0xC,
} OBD2_DTCs_t;

/*******************************************************************************
 *                              External Variables                             *
 *******************************************************************************/
extern CAN_HandleTypeDef     OBD2CAN;
extern UART_HandleTypeDef    SERIAL_DEBUG;
extern const char*           PID_Names[];

/*******************************************************************************
 *                             Functions Declaration                           *
 *******************************************************************************/
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

#endif /* INC_OBD2_H_ */
