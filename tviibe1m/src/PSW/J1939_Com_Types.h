/*
 * J1939_Com_Types.h
 *  Version: 1.0.1
 *  Modify on:  2025/12/02
 *  Added EBC1 Reserved:2 info, Modify TSC1, TCO1, EBC5 Reserved info for spell errors
 *  Created on: 2023/11/15
 *      Author: ABCESC Team
 * Version: 1.0.0
 * Created on: 2023/11/15
 */

#ifndef ABSESCC_J1939_COM_TYPES_H_
#define ABSESCC_J1939_COM_TYPES_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

//****************************************************************************
// @User Files Includes
//****************************************************************************
#include <stdint.h>

//****************************************************************************
// @System Files Includes
//****************************************************************************

//****************************************************************************
// typedef of SAE J1939-71 standard signals
//****************************************************************************
#pragma pack(push, 1)

typedef union __AIR1
{
	uint8_t Data[8];
	struct {
	uint8_t PneumaticSupplyPress;
	uint8_t ParkingAndorTrailerAirPress;
	uint8_t ServiceBrakeCircuit1AirPress;
	uint8_t ServiceBrakeCircuit2AirPress;
	uint8_t AuxEquipmentSupplyPress;
	uint8_t AirSuspensionSupplyPress;
	uint8_t AirCompressorStatus:2;
	uint8_t Reserved:6;
	uint8_t Reserved2;
	} B;
} _AIR1;

typedef union __B
{
	uint8_t Data[8];
	struct {
	uint8_t BrakeAppPress;
	uint8_t BrakePrimaryPress;
	uint8_t BrakeSecondPress;
	uint8_t ParkingBrakeActuator:2;
	uint8_t ParkingBrakeRedWarningSignal:2;
	uint8_t ParkBrakeReleaseInhibitStatus:2;
	uint8_t Reserved:2;

	uint8_t Reserved2[4];
	} B;
} _B;

typedef union __CCVS
{
	uint8_t Data[8];
	struct {
	uint8_t TwoSpeedAxleSwitch:2;
	uint8_t ParkingBrakeSwitch:2;
	uint8_t CruiseCtrlPauseSwitch:2;
	uint8_t ParkBrakeReleaseInhibitRq:2;

	uint16_t WheelBasedVehicleSpeed;

	uint8_t CruiseCtrlActive:2;
	uint8_t CruiseCtrlEnableSwitch:2;
	uint8_t BrakeSwitch:2;
	uint8_t ClutchSwitch:2;

	uint8_t CruiseCtrlSetSwitch:2;
	uint8_t CruiseCtrlCoastSwitch:2;
	uint8_t CruiseCtrlResumeSwitch:2;
	uint8_t CruiseCtrlAccelerateSwitch:2;

	uint8_t CruiseCtrlSetSpeed;

	uint8_t PTOGovernorState:5;
	uint8_t CruiseCtrlStates:3;

	uint8_t EngIdleIncrementSwitch:2;
	uint8_t EngIdleDecrementSwitch:2;
	uint8_t EngTestModeSwitch:2;
	uint8_t EngShutdownOverrideSwitch:2;
	} B;
} _CCVS;

typedef union __EBC1
{
	uint8_t Data[8];
	struct {
	uint8_t ASREngineControlActive:2;
	uint8_t ASRBrakeControlActive:2;
	uint8_t AntiLockBrakingActive:2;
	uint8_t EBSBrakeSwitch:2;

	uint8_t BrakePedalPosition;

	uint8_t ABSOffroadSwitch:2;
	uint8_t ASROffroadSwitch:2;
	uint8_t ASRHillHolderSwitch:2;
	uint8_t TractionControlOverrideSwitch:2;

	uint8_t AcceleratorInterlockSwitch:2;
	uint8_t EngineDerateSwitch:2;
	uint8_t EngineAuxiliaryShutdownSwitch:2;
	uint8_t RemoteAcceleratorEnableSwitch:2;

	uint8_t EngineRetarderSelection;

	uint8_t ABSFullyOperational:2;
	uint8_t EBSRedWarningSignal:2;
	uint8_t AorEBSAmberWarningSignal:2;
	uint8_t ATC_ASRInformationSignal:2;

	uint8_t SourceAddressofControl4BrakeControl;

	uint8_t Reserved:2;
	uint8_t Haltbrakeswitch:2;
	uint8_t TrailerABSStatus:2;
	uint8_t TractorMountedTrailerWarningSignal:2;
	} B;
} _EBC1;

typedef union __EBC2
{
	uint8_t Data[8];
	struct {
	uint16_t FrontAxleSpeed;

	uint8_t FrontAxleLeftWheel;
	uint8_t FrontAxleRightWheel;
	uint8_t RearAxleLeftWheel_1;
	uint8_t RearAxleRightWheel_1;
	uint8_t RearAxleLeftWheel_2;
	uint8_t RearAxleRightWheel_2;
	} B;
} _EBC2;

typedef union __EBC3
{
	uint8_t Data[8];
	struct {
	uint8_t PressureFrontAxleLeftWheel;
	uint8_t PressureFrontAxleRightWheel;
	uint8_t PressureRearAxleLeftWheel_1;
	uint8_t PressureRearAxleRightWheel_1;
	uint8_t PressureRearAxleLeftWheel_2;
	uint8_t PressureRearAxleRightWheel_2;
	uint8_t PressureRearAxleLeftWheel_3;
	uint8_t PressureRearAxleRightWheel_3;
	} B;
} _EBC3;

typedef union __EBC4
{
	uint8_t Data[8];
	struct {
	uint8_t BrakeLiningFrontAxleLeftWheel;
	uint8_t BrakeLiningFrontAxleRightWheel;
	uint8_t BrakeLiningRearAxleLeftWheel_1;
	uint8_t BrakeLiningRearAxleRightWheel_1;
	uint8_t BrakeLiningRearAxleLeftWheel_2;
	uint8_t BrakeLiningRearAxleRightWheel_2;
	uint8_t BrakeLiningRearAxleLeftWheel_3;
	uint8_t BrakeLiningRearAxleRightWheel_3;
	} B;
} _EBC4;

typedef union __EBC5
{
	uint8_t Data[8];
	struct {
	uint8_t BrakeTemperatureWarning:2;
	uint8_t Haltbrakemode:3;
	uint8_t Hillholdermode:3;

	uint8_t FoundationBrakeUse:2;
	uint8_t XBRSystemState:2;
	uint8_t XBRActiveControlMode:4;

	uint8_t XBRAccelerationLimit;

	uint8_t ParkingBrakeActuatorFullyActivated:2;
	uint8_t Reserved:6;

	uint8_t Reserved2[4];
	} B;
} _EBC5;

typedef union __EEC1
{
	uint8_t Data[8];
	struct {
		uint8_t EngTorqueMode:4;
		uint8_t ActlEngPrcntTorqueHighResolution:4;

		uint8_t DriversDemandEngPercentTorque;
		uint8_t ActualEngPercentTorque;

		uint16_t EngSpeed;

		uint8_t SrcAddrssOfCntrllngDvcForEngCtrl;

		uint8_t EngStarterMode:4;
		uint8_t Reserved:4;

		uint8_t EngDemandPercentTorque;
  } B;
} _EEC1;

typedef union __EEC2
{
	uint8_t Data[8];
	struct {
	uint8_t AccelPedal1LowIdleSwitch:2;
	uint8_t AccelPedalKickdownSwitch:2;
	uint8_t RoadSpeedLimitStatus:2;
	uint8_t AccelPedal2LowIdleSwitch:2;

	uint8_t AccelPedalPos1;

	uint8_t EngPercentLoadAtCurrentSpeed;

	uint8_t RemoteAccelPedalPos;

	uint16_t AccelPedalPos2;

	uint8_t VhcleAccelerationRateLimitStatus:2;
	uint8_t Reserved:6;

	uint8_t ActlMaxAvailableEngPercentTorque;

	uint8_t Reserved2;
	} B;
} _EEC2;

typedef union __EEC3
{
	uint8_t Data[8];
	struct {
	uint8_t AccelPedal1LowIdleSwitch:2;
	uint8_t AccelPedalKickdownSwitch:2;
	uint8_t RoadSpeedLimitStatus:2;
	uint8_t AccelPedal2LowIdleSwitch:2;

	uint8_t AccelPedalPos1;

	uint8_t EngPercentLoadAtCurrentSpeed;

	uint8_t RemoteAccelPedalPos;

	uint16_t AccelPedalPos2;

	uint8_t VhcleAccelerationRateLimitStatus:2;
	uint8_t Reserved:6;

	uint8_t ActlMaxAvailableEngPercentTorque;

	uint8_t Reserved2;
	} B;
} _EEC3;

typedef union __ERC1
{
	uint8_t Data[8];
	struct {
	uint8_t TransDrivelineEngaged:2;
	uint8_t TrnsTorqueConverterLockupEngaged:2;
	uint8_t TransShiftInProcess:2;
	uint8_t Reserved:2;

	uint16_t TransOutputShaftSpeed;

	uint8_t PercentClutchSlip;

	uint8_t EngMomentaryOverspeedEnable:2;
	uint8_t ProgressiveShiftDisable:2;
	uint8_t MomentaryEngMaxPowerEnable:2;

	uint16_t TransInputShaftSpeed;

	uint8_t SrcAddrssOfCntrllngDvcFrTrnsCtrl;
	} B;
} _ERC1;

typedef union __ETC1
{
	uint8_t Data[8];
	struct {
	uint8_t TransDrivelineEngaged:2;
	uint8_t TrnsTorqueConverterLockupEngaged:2;
	uint8_t TransShiftInProcess:2;
	uint8_t Reserved:2;

	uint16_t TransOutputShaftSpeed;

	uint8_t PercentClutchSlip;

	uint8_t EngMomentaryOverspeedEnable:2;
	uint8_t ProgressiveShiftDisable:2;
	uint8_t MomentaryEngMaxPowerEnable:2;

	uint16_t TransInputShaftSpeed;

	uint8_t SrcAddrssOfCntrllngDvcFrTrnsCtrl;
	} B;
} _ETC1;

typedef union __ETC2
{
	uint8_t Data[8];
	struct {
	uint8_t TransSelectedGear;
	uint16_t TransActualGearRatio;
	uint8_t TransCurrentGear;
	uint16_t TransRequestedRange;
	uint16_t TransCurrentRange;
	} B;
} _ETC2;

typedef union __ETC4
{
	uint8_t Data[8];
	struct {
	uint8_t TransSynchronizerClutchValue;
	uint8_t TransSynchronizerBrakeValue;
	uint8_t Reservedb[6];
	} B;
} _ETC4;

typedef union __ETC7
{
	uint8_t Data[8];
	struct {
	uint8_t TrnsCrrentRangeDisplayBlankState:2;
	uint8_t TransServiceIndicator:2;
	uint8_t TrnsRqstedRangeDisplayBlankState:2;
	uint8_t TrnsRqstedRangeDisplayFlashState:2;

	uint8_t TransReadyForBrakeRelease:2;
	uint8_t ActiveShiftConsoleIndicator:2;
	uint8_t TransEngCrankEnable:2;
	uint8_t TransShiftInhibitIndicator:2;

	uint8_t TransMode4Indicator:2;
	uint8_t TransMode3Indicator:2;
	uint8_t TransMode2Indicator:2;
	uint8_t TransMode1Indicator:2;

	uint8_t TransRequestedGearFeedback;

	uint8_t TransMode5Indicator:2;
	uint8_t TransMode6Indicator:2;
	uint8_t TransMode7Indicator:2;
	uint8_t TransMode8Indicator:2;

	uint8_t TransWarningIndicator:2;
	uint8_t Reserved:6;

	uint8_t Reservedb[2];
	} B;
} _ETC7;

typedef union __HRW
{
	uint8_t Data[8];
    struct {
    uint16_t FrontAxleLeftWheelSpeed;
    uint16_t FrontAxleRightWheelSpeed;
    uint16_t RearAxleLeftWheelSpeed;
    uint16_t RearAxleRightWheelSpeed;
    } B;
} _HRW;

typedef union __LOI
{
	uint8_t Data[8];
    struct {
	uint8_t BladeCtrlModeSwitch:4;
	uint8_t DesiredGradeOffsetSwitch:4;

	uint8_t BladeAutoModeCmd:4;
	uint8_t LeftBladeCtrlModeOperatorCtrl:4;

	uint8_t RightBladeCtrlModeOperatorCtrl:4;
	uint8_t LftDsiredBladeOffsetOperatorCtrl:4;

	uint8_t RghtDsredBladeOffsetOperatorCtrl:4;
	uint8_t SdshiftBladeCtrlModeOperatorCtrl:4;

	uint8_t SdshftDsrdBldeOffsetOperatorCtrl:4;
	uint8_t Reserved:4;

	uint8_t Reservedb[3];
	} B;
} _LOI;

typedef union __MSF
{
	uint8_t Data[8];
    struct {
	uint8_t Reserved:2;
	uint8_t ASR_ESC_OffroadSwitch:2;
	uint8_t HSA_Switch:2;
	uint8_t Reserved2:2;

	uint8_t EBI_ShutdownSwitch:2;
	uint8_t Reserved3:6;

	uint8_t Reservedb[6];
	} B;
} _MSF;

typedef union __SAS
{
	uint8_t Data[8];
    struct {
    uint16_t SteeringWheelAngle;
    uint8_t SteeringWheelAngleRangeCounter:6;
    uint8_t SteeringWheelAngleRangeCounterType:2;
    uint8_t SteeringAngleSensorErrors;
    uint16_t SteeringWheelAngleRange;

    uint8_t SteeringAngleSensorActiveMode:2;
    uint8_t SteeringAngleSensorCalibrated:2;
    uint8_t SteeringAngleSensorBatteryWarning:2;
    uint8_t :2;
    uint8_t MessageCounter:4;
    uint8_t MessageChecksum:4;
    } B;
} _SAS;

typedef union __SPR
{
	uint8_t Data[8];
    struct {
	uint8_t PneumaticSupplyPressRq;
	uint8_t ParkingAnd_orTrailerAirPressRq;
	uint8_t ServiceBrakeAirPressRqCircuit1;
	uint8_t ServiceBrakeAirPressRqCircuit2;
	uint8_t AuxEquipmentSupplyPressRq;
	uint8_t AirSuspensionSupplyPressRq;
	uint8_t Reservedb[2];
	} B;
} _SPR;

typedef union __TC1
{
	uint8_t Data[8];
    struct {
	uint8_t TransGearShiftInhibitRq:2;
	uint8_t TrnsTrqeConverterLockupDisableRq:2;
	uint8_t DisengageDrivelineRq:2;
	uint8_t TransReverseGearShiftInhibitRq:2;

	uint8_t RequestedPercentClutchSlip;
	uint8_t TransRequestedGear;

	uint8_t DisengageDiffLockRqFrontAxle1:2;
	uint8_t DisengageDiffLockRqFrontAxle2:2;
	uint8_t DisengageDiffLockRqRearAxle1:2;
	uint8_t DisengageDiffLockRqRearAxle2:2;

	uint8_t DisengageDiffLockRqCentral:2;
	uint8_t DisengageDiffLockRqCentralFront:2;
	uint8_t DisengageDiffLockRqCentralRear:2;
	uint8_t :2;

	uint8_t TransMode1:2;
	uint8_t TransMode2:2;
	uint8_t TransMode3:2;
	uint8_t TransMode4:2;

	uint8_t :2;
	uint8_t TransRequestedLaunchGear:4;
	uint8_t TrnsShftSlectorDisplayModeSwitch:2;

	uint8_t TransMode5:2;
	uint8_t TransMode6:2;
	uint8_t TransMode7:2;
	uint8_t TransMode8:2;
	} B;
} _TC1;

typedef union __TCO1
{
	uint8_t Data[8];
    struct {
	uint8_t Driver1WorkingState:3;
	uint8_t Driver2WorkingState:3;
	uint8_t VehicleMotion:2;

	uint8_t Driver1TimeRelatedStates:4;
	uint8_t DriverCardDriver1:2;
	uint8_t VehicleOverspeed:2;

	uint8_t Driver2TimeRelatedStates:4;
	uint8_t DriverCardDriver2:2;
	uint8_t Reserved:2;

	uint8_t SystemEvent:2;
	uint8_t HandlingInformation:2;
	uint8_t TachographPerformance:2;
	uint8_t DirectionIndicator:2;

	uint16_t TachographOutputShaftSpeed;
	uint16_t TachographVehicleSpeed;
	} B;
} _TCO1;

typedef union __TSC1
{
	uint8_t Data[8];
    struct {
    uint8_t EngineOverrideControlMode:2;
    uint8_t EngineRequestedSpeedControlConditions:2;
    uint8_t OverrideControlModePriority:2;
    uint8_t Reserved:2;

    uint16_t EngineRequestedSpeedLimit;

    uint8_t EngineRequestedTorqueLimit;

    uint8_t TSC1TransmissionRate:3;
    uint8_t TSC1ControlPurpose:5;

    uint8_t EngineRequestedTorqueHighResolution:4;
    uint8_t Reserved2:4;

    uint8_t Reserved3:8;

    uint8_t MessageCounter:4;
    uint8_t MessageChecksum:4;
    } B;
} _TSC1;

typedef union __VDC1
{
	uint8_t Data[8];
    struct {
    uint8_t VDCInformationSignal:2;
    uint8_t VDCFullyOperational:2;
    uint8_t VDCbrakelightrequest:2;
    uint8_t :2;

    uint8_t ROPEngineControlactive:2;
    uint8_t ROPBrakeControlactive:2;
    uint8_t YCEngineControlactive:2;
    uint8_t YCBrakeControlactive:2;

    uint8_t Reservedb[6];
    } B;
} _VDC1;

typedef union __VDC2
{
	uint8_t Data[8];
    struct {
    uint16_t SteerWheelAngle;

    uint8_t SteerWheelTurnCounter:6;
    uint8_t SteerWheelAngleSensorType:2;

    uint16_t YawRate;

    uint16_t LateralAcceleration;

    uint8_t LongitudinalAcceleration;
    } B;
} _VDC2;

typedef union __VDHR
{
	uint8_t Data[8];
    struct {
    uint32_t HighResolutionTotalVehicleDistance;
    uint32_t HighResolutionTripDistance;
    } B;
} _VDHR;

typedef union __XBR
{
	uint8_t Data[8];
    struct {
    uint16_t ExternalAccelerationDemand;

    uint8_t XBREBIMode:2;
    uint8_t XBRPriority:2;
    uint8_t XBRCtrlMode:2;
    uint8_t :2;

    uint8_t XBRurgency;
    uint8_t Reservedb[3];
    uint8_t XBRMessageCounter:4;
    uint8_t XBRMessageChecksum:4;
    } B;
} _XBR;
#pragma pack(pop)

#endif /* ABSESCC_J1939_COM_TYPES_H_ */
