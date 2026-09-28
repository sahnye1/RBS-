/**********************************************************************************************
 * 文 件 名: RTE.h
 * 作     者: RBS Team
 * 功能描述: RBS 通讯、底层及应用层参数接口声明
 * 当前版本: V1.0.0
 ***********************************************************************************************/
#ifndef USER_RTE_H_
#define USER_RTE_H_

#include <stdint.h>
#include "J1939_Com_Types.h"
/**/
/*============================================================================================
                                    GLOBAL VARIABLES
============================================================================================*/
/*form PSW*/
extern uint8_t  RTEPSW_Version[5];     // BCD码: 年、月、日、修改当天版本号(默认初值0x01)  example:0126090501
extern uint16_t RTEvBat;               //battery voltage fact:0.1 unit:v
extern uint16_t RTEvIgn;               //ignition voltage fact:0.1 unit:v

extern uint16_t RTEBrkPreX4_14;        //X4_14 pin pressure fact:1 unit:kpa
extern uint16_t RTEBrkPreX4_15;        //X4_15 pin pressure fact:1 unit:kpa
extern uint8_t  RTEfErrPreX4_14;       //X4_14 pin pressure signal error flag  0:no error 1:error
extern uint8_t  RTEfErrPreX4_15;       //X4_15 pin pressure signal error flag  0:no error 1:error

extern uint16_t RTEWheelSpeedFL;       //wheel speed FL fact:1 unit:mm/s
extern uint16_t RTEWheelSpeedFR;       //wheel speed FR fact:1 unit:mm/s
extern uint16_t RTEWheelSpeedRL;       //wheel speed RL fact:1 unit:mm/s
extern uint16_t RTEWheelSpeedRR;       //wheel speed RR fact:1 unit:mm/s
extern uint16_t RTEWheelSpeedXL;       //wheel speed XL fact:1 unit:mm/s
extern uint16_t RTEWheelSpeedXR;       //wheel speed XR fact:1 unit:mm/s

extern uint8_t  RTETaskTime;           //task execute time unit:ms fact:0.1
extern uint8_t  RTEfEESt;              //EEPROM state flag 255:first write EEPROM 86:not first write EEPROM
extern uint8_t  RTEECUType;            //ECU type 0:cab type 1:chassis type

struct PSWErr_Struct
{
	//ABS valve error
	uint8_t RTEfErrInValOpenFL             :1;//front left ABS in valve open load error flag 0:no error 1:error
	uint8_t RTEfErrInValShortFL            :1;//front left ABS in valve short to GND error flag 0:no error 1:error
	uint8_t RTEfErrOutValOpenFL            :1;//front left ABS out valve open load error flag 0:no error 1:error
	uint8_t RTEfErrOutValShortFL           :1;//front left ABS out valve short to GND error flag 0:no error 1:error

	uint8_t RTEfErrInValOpenFR             :1;//front right ABS in valve open load error flag 0:no error 1:error
	uint8_t RTEfErrInValShortFR            :1;//front right ABS in valve short to GND error flag 0:no error 1:error
	uint8_t RTEfErrOutValOpenFR            :1;//front right ABS out valve open load error flag 0:no error 1:error
	uint8_t RTEfErrOutValShortFR           :1;//front right ABS out valve short to GND error flag 0:no error 1:error

	uint8_t RTEfErrInValOpenRL             :1;//rear left ABS in valve open load error flag 0:no error 1:error
	uint8_t RTEfErrInValShortRL            :1;//rear left ABS in valve short to GND error flag 0:no error 1:error
	uint8_t RTEfErrOutValOpenRL            :1;//rear left ABS out valve open load error flag 0:no error 1:error
	uint8_t RTEfErrOutValShortRL           :1;//rear left ABS out valve short to GND error flag 0:no error 1:error

	uint8_t RTEfErrInValOpenRR             :1;//rear right ABS in valve open load error flag 0:no error 1:error
	uint8_t RTEfErrInValShortRR            :1;//rear right ABS in valve short to GND error flag 0:no error 1:error
	uint8_t RTEfErrOutValOpenRR            :1;//rear right ABS out valve open load error flag 0:no error 1:error
	uint8_t RTEfErrOutValShortRR           :1;//rear right ABS out valve short to GND error flag 0:no error 1:error

	uint8_t RTEfErrInValOpenXL             :1;//additional left ABS in valve open load error flag 0:no error 1:error
	uint8_t RTEfErrInValShortXL            :1;//additional left ABS in valve short to GND error flag 0:no error 1:error
	uint8_t RTEfErrOutValOpenXL            :1;//additional left ABS out valve open load error flag 0:no error 1:error
	uint8_t RTEfErrOutValShortXL           :1;//additional left ABS out valve short to GND error flag 0:no error 1:error

	uint8_t RTEfErrInValOpenXR             :1;//additional right ABS in valve open load error flag 0:no error 1:error
	uint8_t RTEfErrInValShortXR            :1;//additional right ABS in valve short to GND error flag 0:no error 1:error
	uint8_t RTEfErrOutValOpenXR            :1;//additional right ABS out valve open load error flag 0:no error 1:error
	uint8_t RTEfErrOutValShortXR           :1;//additional right ABS out valve short to GND error flag 0:no error 1:error

	uint8_t RTEfErrX3_7InValveOpen;            //X3_7 In valve open error flag  0: no error 1: error
	uint8_t RTEfErrX3_7InValveShort;           //X3_7 In valve short error flag  0: no error 1: error
	uint8_t RTEfErrX3_10OutValveOpen;          //X3_10 Out valve open error flag  0: no error 1: error
	uint8_t RTEfErrX3_10OutValveShort;         //X3_10 Out valve short error flag  0: no error 1: error
	uint8_t RTEfErrX4_16InValveOpen;           //X4_16 In valve open error flag  0: no error 1: error
	uint8_t RTEfErrX4_16InValveShort;          //X4_16 In valve short error flag  0: no error 1: error
	uint8_t RTEfErrX2_16OutValveOpen;          //X2_16 Out valve open error flag  0: no error 1: error
	uint8_t RTEfErrX2_16OutValveShort;         //X2_16 Out valve short error flag  0: no error 1: error

	//speed sensor error
	uint8_t RTEfErrWssOpenFL               :1;//FL wheel speed sensor open error flag  0:no error 1:error
	uint8_t RTEfErrWssOpenFR               :1;//FR wheel speed sensor open error flag  0:no error 1:error
	uint8_t RTEfErrWssOpenRL               :1;//RL wheel speed sensor open error flag  0:no error 1:error
	uint8_t RTEfErrWssOpenRR               :1;//RR wheel speed sensor open error flag  0:no error 1:error
	uint8_t RTEfErrWssOpenXL               :1;//XL wheel speed sensor open error flag  0:no error 1:error
	uint8_t RTEfErrWssOpenXR               :1;//XR wheel speed sensor open error flag  0:no error 1:error

	uint8_t RTEfErrWssShortFL              :1;//FL wheel speed sensor short error flag  0:no error 1:error
	uint8_t RTEfErrWssShortFR              :1;//FR wheel speed sensor short error flag  0:no error 1:error
	uint8_t RTEfErrWssShortRL              :1;//RL wheel speed sensor short error flag  0:no error 1:error
	uint8_t RTEfErrWssShortRR              :1;//RR wheel speed sensor short error flag  0:no error 1:error
	uint8_t RTEfErrWssShortXL              :1;//XL wheel speed sensor short error flag  0:no error 1:error
	uint8_t RTEfErrWssShortXR              :1;//XR wheel speed sensor short error flag  0:no error 1:error

	uint8_t RTEfErrWssGapFL                :1;//FL wheel speed sensor gap error flag  0:no error 1:error
	uint8_t RTEfErrWssGapFR                :1;//FR wheel speed sensor gap error flag  0:no error 1:error
	uint8_t RTEfErrWssGapRL                :1;//RL wheel speed sensor gap error flag  0:no error 1:error
	uint8_t RTEfErrWssGapRR                :1;//RR wheel speed sensor gap error flag  0:no error 1:error
	uint8_t RTEfErrWssGapXL                :1;//XL wheel speed sensor gap error flag  0:no error 1:error
	uint8_t RTEfErrWssGapXR                :1;//XR wheel speed sensor gap error flag  0:no error 1:error


	uint8_t RTEfErrVbatLo                  :1;//battery voltage low error flag 0:no error 1:error
	uint8_t RTEfErrVbatHi                  :1;//battery voltage high error flag 0:no error 1:error
        
        uint8_t RTEfErrVpwrLo                  :1;//vpower voltage low error flag 0:no error 1:error
        uint8_t RTEfErrVpwrHi                  :1;//vpower voltage high error flag 0:no error 1:error

	uint8_t RTEfErrEEprom                  :1;//EEprom option error flag 0:no error 1:error
	uint8_t RTEfErrEEpromOutBnd            :1;//write EEprom out bound flag 0:no error 1:error
	uint8_t RTEfErrRelay		       :1;//Relay error flag 0:no error 1:error
        uint8_t RTEfErrDriveChip	       :1;//Drive chip error flag 0:no error 1:error
};

extern struct PSWErr_Struct RTEfPSWErr;

typedef struct 
{
      uint32_t msgID;                             //can recive message ID
      uint8_t msgRcvFlag;                         //message rcv flag
      uint8_t msg[8];                             //message
}Can_recv_msg_st;
/*PSW*/

/*form COM*/
extern uint8_t  RTECOM_Version[5];                 // BCD码: 年、月、日、修改当天版本号(默认初值0x01)  example:0226090501
extern uint16_t RTEYawRate;                        //yaw rate fact: 0.01 unit:degree/s offset 30000 direction clockwise: >30000 direction anticlockwise: <30000
extern uint16_t RTELatAcc;                         //lateral acceleration fact: 0.01 unit: m/s^2 offset 2100 direction left: >2100 dirction right: <2100
extern uint16_t RTELongiAcc;                       //longitudinal acceleration fact: 0.01 unit: m/s^2 offset 2100 direction up: >2100 direction down: <2100

extern uint16_t RTEVehPitchAngle;                  //vehicle pitch angle fact: 0.01 unit: degree
extern uint8_t  RTEVehAngleDir;                    //vehicle pitch angle direction 0:elevation angle or level 1:depression angle

extern uint8_t  RTEYawRateDirecton;                //yaw rate direction 0:yaw clockwise or go straight 1:yaw anticlockwise
extern uint8_t  RTELatAccDirection;                //lateral acceleration direction 0:left low or level 1:right low
extern uint8_t  RTELongiDirection;                 //longitudinal acceleration direction 0:head up or level 1:head down

extern uint8_t  RTEAccelPedal;                     //accelerator pedal position 0 to 100%
extern uint16_t RTEEngineSpeed;                    //engine speed fact 0.125 unit:rpm
extern uint8_t  RTEGearPosition;                   //gear position ETC2 value
extern uint8_t  RTEGearSelectedPosition;           //gear selected position ETC2 value
extern uint8_t  RTEKeyPosition;                    //key position 0:OFF 1:ACC/ON/START
extern uint16_t RTEGasCylinderPreF;                //front axle gas cylinder pressure fact:1 unit:kpa. if no gas cylinder pressure,set to 0XFF
extern uint16_t RTEGasCylinderPreR;                //rear axle gas cylinder pressure fact:1 unit:kpa. if no gas cylinder pressure,set to 0XFF
extern uint8_t  RTETransReadyForBrakeRelease;      //transmission ready for brake release from CAN message ETC7 0:not ready 1:ready
extern uint16_t RTEExVehicleSpeed;                 //external vehicle speed fact:0.01 unit:km/h
extern uint8_t  RTEParkBrkSt;                      //park brake state 0:park brake not active 1:park brake active
extern uint16_t RTEVSWAngel;                       //steering wheel angel fact: 1/1024 unit: rad offset:-31.374rad
extern uint8_t  RTEDriversDemandEngPercentTorque;  //Drivers Demand Engine Torque. UNIT:%
extern uint8_t  RTEActualEngPercentTorque;         //Actual Engine Torque. UNIT:%
extern uint8_t  RTEComHaltBrakeSwitch;             //halt brake switch state signal from CAN. 0:off 1:on
extern uint8_t  RTEComABSOffRoadSwitch;            //ABS off road switch state signal from CAN. 0:off 1:on
extern uint8_t  RTEComESCOffSwitch;                //ESC off switch state signal from CAN. 0:off 1:on
extern uint8_t  RTEComHSASwitch;                   //HSA switch state signal from CAN. 0:off 1:on
extern uint8_t  RTEComASRSwitch;                   //ASR switch state signal from CAN. 0:off 1:on

struct COMErr_Struct
{
    uint8_t  RTEfErrAccelPedal                  :1;//accelerator pedal signal error flag 0:no error 1:error
    uint8_t  RTEfErrEngineTorque                :1;//engine torque error flag 0:no error 1:error
    uint8_t  RTEfErrEngineSpeed                 :1;//engine speed error flag 0:no error 1:error
    uint8_t  RTEfErrGearPosition                :1;//gear position error flag 0:no error 1:error
    uint8_t  RTEfErrKeyPosition                 :1;//key position error flag 0: no error 1:error
    uint8_t  RTEfErrGasCylinderPreF             :1;//front axle gas cylinder pressure error flag 0: no error 1:error
    uint8_t  RTEfErrGasCylinderPreR             :1;//rear axle gas cylinder pressure error flag 0: no error 1:error
    uint8_t  RTEfErrTransReadyForBrakeRelease   :1;//error flag 0:no error 1:error
    uint8_t  RTEfErrExVehicleSpeed              :1;//external vehicle speed error flag  0:no error 1:error
    uint8_t  RTEfErrParkBrkSt                   :1;//error flag 0:no error 1:error
    uint8_t  RTEfErrExternalAccelerationDemand  :1;//error flag 0:no error 1:error
    
    uint8_t RTEfErrInValOpenF                   :1;//front InValOpenF
    uint8_t RTEfErrInValShortF                  :1;//front InValShortF
    uint8_t RTEfErrOutValOpenF                  :1;//front OutValOpenF
    uint8_t RTEfErrOutValShortF                 :1;//front OutValShortF
    uint8_t RTEfErrInValOpenR                   :1;//rear InValOpenF
    uint8_t RTEfErrInValShortR                  :1;//rear InValShortF
    uint8_t RTEfErrOutValOpenR                  :1;//rear OutValOpenF
    uint8_t RTEfErrOutValShortR                 :1;//rear OutValShortF
    
    uint8_t  RTEfErrSASNoCalib                  :1;//SAS system no calibration error flag  0: no error 1: error
    uint8_t  RTEfErrSASLost                     :1;//SAS Module Lost 0:no error 1:error
    uint8_t  RTEfErrSASInner                    :1;//SAS Module signal error 0:no error 1:error
    uint8_t  RTEfErrSASCANID                    :1;//SAS Module CAN ID 0:no error 1:error
    uint8_t  RTEfErrSASSgn                      :1;//SAS Module signal error 0:no error 1:error
    uint8_t  RTEfErrSASLowVol                   :1;//SAS Module low voltage error 0:no error 1:error    
    uint8_t  RTEfErrSASHiVol                    :1;//SAS Module high voltage error 0:no error 1:error
    
    uint8_t  RTEfErrESCMNoCalib                 :1;//ESCM system no calibration error flag  0: no error 1: error
    uint8_t  RTEfErrESCMLost                    :1;//ESCM Module Lost 0:no error 1:error
    uint8_t  RTEfErrESCMInner                   :1;//ESCM Module signal error 0:no error 1:error
    uint8_t  RTEfErrESCMCANID                   :1;//ESCM Module CAN ID 0:no error 1:error
    uint8_t  RTEfErrESCMSgn                     :1;//ESCM Module signal error 0:no error 1:error
    uint8_t  RTEfErrESCMLowVol                  :1;//ESCM Module low voltage error 0:no error 1:error    
    uint8_t  RTEfErrESCMHiVol                   :1;//ESCM Module high voltage error 0:no error 1:error
    uint8_t  RTEfEEC1MsgRxErr                   :1;//EEC1 message receive error 0:no error 1:error
    uint8_t  RTEfEEC2MsgRxErr                   :1;//EEC2 message receive error 0:no error 1:error
    uint8_t  RTEfETC2MsgRxErr                   :1;//ETC2 message receive error 0:no error 1:error
    uint8_t  RTEfETC7MsgRxErr                   :1;//ETC7 message receive error 0:no error 1:error
    uint8_t  RTEfXBRAEBSMsgRxErr                :1;//XBRAEBS message receive error 0:no error 1:error
    uint8_t  RTEfPsToEBSMsgRxErr                :1;//PsToEBS message receive error 0:no error 1:error
};

extern struct COMErr_Struct RTEfCOMErr;
extern uint8_t  RTEfSysUseSt;                      //system use state 0:normal state 1:FCT test 2:OEM EOL test
extern uint8_t  RTEReqClearDTC;                    //Clear All DTC(erase flash)0:no request 1:request
extern uint8_t  RTEfConfig;                        //configure vehicle parameters flag 0:no configure 1:configure
extern uint8_t  RTEfConfig2Def;                    //configure vehicle parameters to default flag 0:no configure 1:configure
extern uint8_t  RTEfBin2EE;                        //configure bin file parameters to eeprom 0:no configure 1:configure
extern uint8_t  RTEfClearCVW;                      //clear calculate vehicle weight flag
extern uint8_t  RTEfBoot;                          //Boot Start
//****************************************************************************
//RTEVMode:
//MODE_4S4M	    0
//MODE_6S6M	    1
//****************************************************************************
extern uint8_t RTEVMode;                            //vehicle mode r/w

//****************************************************************************
//RTETrMode:
//TRAILER_NONE 	0
//TRAILER    	1
//****************************************************************************
extern uint8_t RTETrMode;                            //trailer mode r/w
extern uint8_t RTEfDriveAxle;                        //0:rear axle 1:additional axle 2:front axle
extern uint8_t RTEfSASPlugDirection;                 //r/w 0:down 1:up
extern uint8_t RTEfESCCPlugDirection;                //ABSESC Plug Direction 0:front 1:back
extern uint8_t RTEPSPosNumber;                       //pressure sensor position and number 0:1rear 1:1front 2:two sensor

extern uint8_t RTEABSOFFSWST;                        //ABS Off Switch State r/w 0:function on 1:function off
extern uint8_t RTEASROFFSWST;                        //ASR Off Switch State r/w 0:function on 1:function off
extern uint8_t RTEHSAONSWST;                         //HSA On Switch State r/w 0:function off 1:function on
extern uint8_t RTEESCOFFSWST;                        //ESC Switch State r/w 0:function on 1:function off
extern uint8_t RTEHALTBRAKESWST;                     //Halt Brake Switch State r/w 0:function off 1:function on

extern uint8_t RTEfABSEn;                            //ABS Function Enable Flag r/w 0:not enable, 1:enable
extern uint8_t RTEfASREn;                            //ASR Function Enable Flag r/w 0:not enable, 1:enable
extern uint8_t RTEfHSAEn;                            //HSA Function Enable Flag r/w 0:not enable, 1:enable
extern uint8_t RTEfESCEn;                            //ESC Function Enable Flag r/w 0:not enable, 1:enable
extern uint8_t RTEfHaltBrakeEn;                      //Halt Brake Function Enable Flag r/w 0:not enable, 1:enable
extern uint8_t RTEfXBREn;                            //XBR Function Enable Flag r/w 0:not enable, 1:enable

extern uint16_t RTEVBase[4];                         //vehicle base r/w 1:1mm轴距
extern uint8_t  RTEVWSToothCnt;                      //vehicle wheel speed sensor tooth counter  r/w
extern uint16_t RTEVWRDiameter;                      //vehicle wheel roll diameter r/w 1:1mm
extern uint8_t  RTEMaxSpeed;                         //vehicle max speed r/w 1:1km/h
extern uint8_t  RTESteerRatio;                       //Steering ratio 0:20.5;1:23;2:22.2-26.22;3:17-20;4:17.6-20.8;5:18.3
extern uint8_t  RTEEngTqLmtTest;                     //engine torque limit test.range:-125 to 125% fact:1%/bit offset:-125% 255:invalid
extern uint16_t RTEBrkPreF;                          //front axle pressure fact:1 unit:kpa. if just use 1 rear axle pressure sensor , RTEBrkPreF = RTEBrkPreR
extern uint16_t RTEBrkPreR;                          //rear axle pressure fact:1 unit:kpa
extern uint8_t  RTEfErrPreF;                         //front axle pressure signal error flag  0:no error 1:error
extern uint8_t  RTEfErrPreR;                         //rear axle pressure signal error flag  0:no error 1:error
extern uint8_t  RTEXBRDbgLadF;                       //XBR debug ladder, front bridge ,0-50
extern uint8_t  RTEXBRDbgLadR;                       //XBR debug ladder, rear bridge ,0-50
extern uint8_t  RTEXBRDbgLadX;                       //XBR debug ladder, additional bridge ,0-50
extern uint8_t  RTEXBRDbgLadTr;                      //XBR debug ladder, trailer ,0-50

extern uint8_t RTEfTestMode;                         //test mode 0:normal mode 1:factory test mode 2:OEM test mode
extern uint8_t RTEfAddAxlePos;                       //additional axle position used in MODE_6S6M 0:middle 1:rear
extern uint8_t  RTEfValDbgCtrl;                      //valve debug control flag 0:not control 1:control
extern uint16_t RTEDbgPreF;                          //debug front pressure request fact:1 unit:kpa
extern uint16_t RTEDbgPreR;                          //debug front pressure request fact:1 unit:kpa

// 0: RTEBrkPreF <-> RTEBrkPreX4_14, RTEBrkPreR <-> RTEBrkPreX4_14, RTEfErrPreF <-> RTEfErrPreX4_14
// 1: RTEBrkPreF <-> RTEBrkPreX4_14, RTEBrkPreR <-> RTEBrkPreX4_14, RTEfErrPreR <-> RTEfErrPreX4_14
// 2: RTEBrkPreF <-> RTEBrkPreX4_15, RTEBrkPreR <-> RTEBrkPreX4_15, RTEfErrPreF <-> RTEfErrPreX4_15
// 3: RTEBrkPreF <-> RTEBrkPreX4_15, RTEBrkPreR <-> RTEBrkPreX4_15, RTEfErrPreR <-> RTEfErrPreX4_15
// 4: RTEBrkPreF <-> RTEBrkPreX4_14, RTEBrkPreR <-> RTEBrkPreX4_15, RTEfErrPreF <-> RTEfErrPreX4_14, RTEfErrPreR <-> RTEfErrPreX4_15
// 5: RTEBrkPreF <-> RTEBrkPreX4_15, RTEBrkPreR <-> RTEBrkPreX4_14, RTEfErrPreF <-> RTEfErrPreX4_15, RTEfErrPreR <-> RTEfErrPreX4_14
extern uint8_t RTEfPressCfg;

//0: RTEfInValveF<->RTEfInValveX3_7   RTEfOutValveF<->RTEfOutValveX3_10  RTEfInValveR<->RTEfInValveX4_16  RTEfOutValveR<->RTEfOutValveX2_16
//1: RTEfInValveF<->RTEfInValveX4_16  RTEfOutValveF<->RTEfOutValveX2_16  RTEfInValveR<->RTEfInValveX3_7   RTEfOutValveR<->RTEfOutValveX3_10
//2: RTEfInValveF<->RTEfInValveX3_7   RTEfOutValveF<->RTEfOutValveX2_16  RTEfInValveR<->RTEfInValveX4_16  RTEfOutValveR<->RTEfOutValveX3_10
//3: RTEfInValveF<->RTEfInValveX4_16  RTEfOutValveF<->RTEfOutValveX3_10  RTEfInValveR<->RTEfInValveX3_7   RTEfOutValveR<->RTEfOutValveX2_16
extern uint8_t RTEfFR_InOutValCfg;

extern uint8_t DATA_0xF180[7];
extern uint8_t DATA_0xF181[7];
extern uint8_t DATA_0xF182[7];
extern uint8_t DATA_0xF183[42];
extern uint8_t DATA_0xF184[42];
extern uint8_t DATA_0xF185[42];
extern uint8_t DATA_0xF187[13];
extern uint8_t DATA_0xF18A[8];
extern uint8_t DATA_0xF18B[4];
extern uint8_t DATA_0xF18C[12];
extern uint8_t DATA_0xF190[17];
extern uint8_t DATA_0xF193[6];
extern uint8_t DATA_0xF195[6];
extern uint8_t DATA_0xF197[9];
extern uint8_t DATA_0xF192[5];
extern uint8_t DATA_0xF194[7];

/*configure parameters*/
extern uint8_t DATA_0x5010[8];           //base
extern uint8_t DATA_0x5011[1];           //tooth counter
extern uint8_t DATA_0x5012[2];           //diameter
extern uint8_t DATA_0x5013[2];           //vehicle mode
extern uint8_t DATA_0x5014[2];           //max speed
extern uint8_t DATA_0x5015[1];           //ESC Switch Type
extern uint8_t DATA_0x5016[1];           //reserved
extern uint8_t DATA_0x5017[1];           //ABS Function Switch 0:not enable, 1:enable
extern uint8_t DATA_0x5018[1];           //ESC Function Switch 0:not enable, 1:enable
extern uint8_t DATA_0x5019[1];           //ASR Function Switch 0:not enable, 1:enable
extern uint8_t DATA_0x501A[1];           //HSA Function Switch 0:not enable, 1:enable
extern uint8_t DATA_0x501B[1];           //XBR Function Switch 0:not enable, 1:enable
extern uint8_t DATA_0x501C[1];           //Steering ratio 0:20.5;1:23;2:22.2-26.22;3:17-20;4:17.6-20.8;5:18.3
extern uint8_t DATA_0x501D[1];           //Communication Baud Rate 0:250k 1:500k
extern uint8_t DATA_0x501E[1];           //SAS Type
extern uint8_t DATA_0x501F[1];           //ESC Type
extern uint8_t DATA_0x5678[1];           //configure parameters to default values
extern uint8_t DATA_0x5200[154];         //configure parameters when change the ECU
extern uint8_t DATA_0x5201[150];         // configure calibration parameters

extern uint8_t RTEfABSOffSwitchSrc;
extern uint8_t RTEfASROffSwitchSrc;
extern uint8_t RTEfESCOffSwitchSrc;
extern uint8_t RTEfHSASwitchSrc;

extern uint8_t RTEfABSOffSwitchType;
extern uint8_t RTEfASROffSwitchType;
extern uint8_t RTEfESCOffSwitchType;
extern uint8_t RTEfHSASwitchType;

extern uint16_t RTEGroundTest;           //Ground test

//extern configure bin values
typedef union
{
    uint8_t Data[154];
} BINPARA;
extern BINPARA BINPARA0;
extern _XBR RTEMSGXBR;
/*COM*/

/*form ASW*/
extern uint8_t RTEASW_Version[5];         // BCD码: 年、月、日、修改当天版本号(默认初值0x01)  example:0326090501
extern uint8_t RTEDTC[224];
extern uint8_t RTEfCalib;                 //calibration bin file parameters to ram 0:no configure 1:configure
extern uint8_t RTEfValWssTestForbit;
extern uint8_t RTEfCAN0TxForbid;
extern uint8_t RTEfCAN0RxForbid;
extern uint8_t RTEComBaudRate;            //Communication Baud Rate 0:250k 1:500k
extern uint8_t RTEfDbgMsgSW, RTEfDbgMsgSW1, RTEfDbgMsgSW2, RTEfDbgMsgSW3;

extern _EBC1 RTEMSGEBC1;
extern _EBC2 RTEMSGEBC2;
extern _VDC1 RTEMSGVDC1;
extern _VDC2 RTEMSGVDC2;

extern _TSC1 RTEMSGTSC1_E;
extern _EBC5 RTEMSGEBC5;
extern _HRW RTEMSGHRW;
/*ASW*/

/*============================================================================================
                                    GLOBAL FUNCTIONS
     @Description   This file contains functions that use for different modules.
============================================================================================*/
/*form PSW*/
//ABS valve control
//函数 InValActFL() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void InValActFL(uint16_t period,uint16_t htime);//左前ABS进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActFL() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActFL(uint16_t period,uint16_t htime);//左前ABS排气阀动作函数 fact:0.1 unit: ms

//函数 InValActFR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void InValActFR(uint16_t period,uint16_t htime);//右前ABS进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActFR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActFR(uint16_t period,uint16_t htime);//右前ABS排气阀动作函数 fact:0.1 unit: ms

//函数 InValActRL() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void InValActRL(uint16_t period,uint16_t htime);//左后ABS进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActRL() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActRL(uint16_t period,uint16_t htime);//左后ABS排气阀动作函数 fact:0.1 unit: ms

//函数 InValActRR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void InValActRR(uint16_t period,uint16_t htime);//右后ABS进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActRR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActRR(uint16_t period,uint16_t htime);//右后ABS排气阀动作函数 fact:0.1 unit: ms

//函数 InValActXL() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void InValActXL(uint16_t period,uint16_t htime);//中桥左ABS进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActXL() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActXL(uint16_t period,uint16_t htime);//中桥左ABS排气阀动作函数 fact:0.1 unit: ms

//函数 InValActXR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void InValActXR(uint16_t period,uint16_t htime);//中桥右ABS进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActXR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActXR(uint16_t period,uint16_t htime);//中桥右ABS排气阀动作函数 fact:0.1 unit: ms

//函数 InValActX3_7() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void  InValActX3_7(uint16_t period,uint16_t htime);//X3_7端口进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActX3_10() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActX3_10(uint16_t period, uint16_t htime);//X3_10端排气阀动作函数 fact:0.1 unit: ms

//函数 InValActX4_16() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void  InValActX4_16(uint16_t period, uint16_t htime);//X4_16端进气阀动作函数 fact:0.1 unit: ms

//函数 OutValActX2_16() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActX2_16(uint16_t period, uint16_t htime);//X2_16端排气阀动作函数 fact:0.1 unit: ms

extern void RelayCtrl(uint8_t state);//继电器控制函数 0:off 1:on
extern void UBCtrl(uint8_t state);   //UB电压控制函数 0:off 1:on
/* 
 * 功能: can过滤ID列表配置
 * 参数:
 *    canX: CAN类型, 0 - 底盘CAN(CAN0); 1 - 调试CAN(CAN1);
 *    canIDList: can ID数组 (标准帧 ≤0x7FF, 扩展帧 >0x7FF)
 *    idCount: CAN ID数量
 *    transDirect: 0 - 接收列表(配置过滤器); 1 - 发送列表(CAN0 发送白名单, CAN1 忽略);
 * 注意: 必须在 can_init() / board_init() 之前调用, 否则 CAN 以空过滤器初始化,
 *       Can0_Init/Can1_Init 会直接返回失败, 导致运行时收发全部无效果。
 */
extern void can_info_cfg(uint8_t canX, uint32_t* canIDList, uint8_t idCount, uint8_t transDirect);
/*
 * 功能：公共can比特率设置
 * 参数：bitRateFlag：0 - 1MHz; 1 - 500KHz; 2 - 250KHz; 3 - 125KHz; 其他 - 500KHz;
 */
extern void can0_bitrate_set(uint8_t bitRateFlag);
/* 
 * 功能: 底盘can发送函数
 * 参数:
 *    msgId: 发送ID，Can0_msgID_e中已配置的TX id
 *    msg: 发送报文
 *    msgLen：发送报文长度，最大为8
 * 返回值：0 -- 发送成功; 1 -- 参数错误; 2 -- can通信失败; 
 */
extern uint8_t can0_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen);
/* 
 * 功能: 调试can发送函数
 * 参数:
 *    msgId: 发送ID，Can1_msgID_e中已配置的TX id
 *    msg: 发送报文
 *    msgLen：发送报文长度，最大为8
 * 返回值：0 -- 发送成功; 1 -- 参数错误; 2 -- can通信失败; 
 */
extern uint8_t can1_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen);
/* 
 * 功能: 底盘can获取接收报文
 * 参数:
 *    recvMsg: 接收报文buf，接收ID为索引
 *    msgMaxCnt：接收报文结构体数据的容量，建议为底盘can接收ID的数目
 * 返回值：实际接收到报文的接收ID数量
 */
extern uint8_t can0_getMsg(Can_recv_msg_st *recvMsg, uint8_t msgMaxCnt);
/* 
 * 功能: 调试can获取接收报文函数
 * 参数:
 *    recvMsg: 接收报文buf，接收ID为索引
 *    msgMaxCnt：接收报文结构体数据的容量，建议为调试can接收ID的数目
 * 返回值：实际接收到报文的接收ID数量
 */
extern uint8_t can1_getMsg(Can_recv_msg_st *recvMsg, uint8_t msgMaxCnt);

/*
 * 功能：获取PSW库的版本信息
 * 参数：
 *      verStr: 回参buf，返回PSW的版本信息
 *      bufLen: VerStr的最大长度
 * 返回值：0 - 参数错误; 其他 - PSW库版本信息的真实长度;
 */
extern uint8_t psw_version_get(uint8_t *verStr, uint8_t bufLen);
/*
 * 功能：故障码数据写入flash
 * 参数：
 *     writeAddr：写入的相对地址，单位为字节，范围为[0 ~ 127]
 *     writeData：写入的数据
 *     dataLen：写入的数据的字节数，writeAddr + dataLen < 128
 * 返回：0 - 写入失败; 其他 - 写入数据的字节数;
 */
extern uint32_t ERR_CODE_write(uint32_t writeAddr, uint8_t *writeData, uint32_t dataLen);
/*
 * 功能：从flash中读取故障码数据
 * 参数：
 *     readAddr：读取的相对地址，单位为字节，范围为[0 ~ 127]
 *     readBufferPtr：回参，读取数据的buffer
 *     dataLen：需要读取的数据的字节数，readAddr + dataLen < 128
 * 返回：0 - 读取失败; 0xFFFFFFFF - 读取的数据区为空; 其他 - 读取的数据的字节数;
 */
extern uint32_t ERR_CODE_read(uint32_t readAddr, uint8_t* readBufferPtr, uint32_t dataLen);

/*
 * 功能：应用层配置参数写入flash
 * 参数：
 *     writeAddr：写入的相对地址，单位为字节，范围为[0 ~ 127]
 *     writeData：写入的数据
 *     dataLen：写入的数据的字节数，writeAddr + dataLen < 128
 * 返回：0 - 写入失败; 其他 - 写入数据的字节数;
 */
extern uint32_t ASW_cfg_write(uint32_t writeAddr, uint8_t *writeData, uint32_t dataLen);
/*
 * 功能：从flash中读取应用层配置参数
 * 参数：
 *     readAddr：读取的相对地址，单位为字节，范围为[0 ~ 127]
 *     readBufferPtr：回参，读取数据的buffer
 *     dataLen：需要读取的数据的字节数，readAddr + dataLen < 128
 * 返回：0 - 读取失败; 0xFFFFFFFF - 读取的参数区为空; 其他 - 读取的参数的字节数;
 */
extern uint32_t ASW_cfg_read(uint32_t readAddr, uint8_t* readBufferPtr, uint32_t dataLen);
/*
 * 功能：底层配置参数写入flash
 * 参数：
 *     writeAddr：写入的相对地址，单位为字节，范围为[0 ~ 127]
 *     writeData：写入的数据
 *     dataLen：写入的数据的字节数，writeAddr + dataLen < 128
 * 返回：0 - 写入失败; 其他 - 写入数据的字节数;
 */
extern uint32_t PSW_cfg_write(uint32_t writeAddr, uint8_t *writeData, uint32_t dataLen);
/*
 * 功能：从flash中读取底层配置参数
 * 参数：
 *     readAddr：读取的相对地址，单位为字节，范围为[0 ~ 127]
 *     readBufferPtr：回参，读取数据的buffer
 *     dataLen：需要读取的数据的字节数，readAddr + dataLen < 128
 * 返回：0 - 读取失败; 0xFFFFFFFF - 读取的参数区为空; 其他 - 读取的参数的字节数;
 */
extern uint32_t PSW_cfg_read(uint32_t readAddr, uint8_t* readBufferPtr, uint32_t dataLen);
/*
 * 功能：通讯层配置参数写入flash
 * 参数：
 *     writeAddr：写入的相对地址，单位为字节，范围为[0 ~ 127]
 *     writeData：写入的数据
 *     dataLen：写入的数据的字节数，writeAddr + dataLen < 128
 * 返回：0 - 写入失败; 其他 - 写入数据的字节数;
 */
extern uint32_t COM_cfg_write(uint32_t writeAddr, uint8_t *writeData, uint32_t dataLen);
/*
 * 功能：从flash中读取通讯层配置参数
 * 参数：
 *     readAddr：读取的相对地址，单位为字节，范围为[0 ~ 127]
 *     readBufferPtr：回参，读取数据的buffer
 *     dataLen：需要读取的数据的字节数，readAddr + dataLen < 128
 * 返回：0 - 读取失败; 0xFFFFFFFF - 读取的参数区为空; 其他 - 读取的参数的字节数;
 */
extern uint32_t COM_cfg_read(uint32_t readAddr, uint8_t* readBufferPtr, uint32_t dataLen);
/*
 * 功能：升级配置参数写入flash
 * 参数：
 *     writeAddr：写入的相对地址，单位为字节，范围为[0 ~ 127]
 *     writeData：写入的数据
 *     dataLen：写入的数据的字节数，writeAddr + dataLen < 128
 * 返回：0 - 写入失败; 其他 - 写入数据的字节数;
 */
extern uint32_t BOOT_cfg_write(uint32_t writeAddr, uint8_t *writeData, uint32_t dataLen);
/*
 * 功能：从flash中读取升级配置参数
 * 参数：
 *     readAddr：读取的相对地址，单位为字节，范围为[0 ~ 127]
 *     readBufferPtr：回参，读取数据的buffer
 *     dataLen：需要读取的数据的字节数，readAddr + dataLen < 128
 * 返回：0 - 读取失败; 0xFFFFFFFF - 读取的参数区为空; 其他 - 读取的参数的字节数;
 */
extern uint32_t BOOT_cfg_read(uint32_t readAddr, uint8_t* readBufferPtr, uint32_t dataLen);

/* 函数:InLowSideSwX4_16()
 * 描述: X4_16端进气阀低边开关控制。
 * 参数: 
 *      sw: 开关值 0 - 断开; 其他 - 闭合;
 */
void InLowSideSwX4_16(uint32_t sw);

/* 函数: InLowSideStX4_16()
 * 描述:  X4_16端进气阀低边开关状态。
 * 参数: NONE
 * 返回值: 开关状态值 0 - 断开; 其他 - 闭合;
 */
uint32_t InLowSideStX4_16(void);

/* 函数: OutLowSideSwX2_16()
 * 描述: X2_16端排气阀低边开关控制。
 * 参数: 
 *      sw: 开关值 0 - 断开; 其他 - 闭合;
 */
void OutLowSideSwX2_16(uint8_t sw);

/* 函数: OutLowSideStX2_16()
 * 描述: X2_16端排气阀低边开关状态。
 * 参数: NONE
 * 返回值: 开关状态值 0 - 断开; 其他 - 闭合;
 */
uint32_t OutLowSideStX2_16(void);

/*
 * 功能：X1的15状态设置（ABS故障灯控制开关，P18.4）
 * 参数：setValue：0 - 关闭; 1 - 打开;
 */
extern void X1_15_staSet(uint8_t setValue);
/*
 * 功能：接插件指定引脚开关状态获取
 * 参数：xNum：接插件编号
 *       xPin : 接插件的引脚编号
 *      有效的参数取值组合为(xNum, xPin)：(1, 11) - X1的11脚，挂车ABS功能开关状态获取; (3, 11) - X3的11脚,ASR和ESC功能开关状态获取; (3, 14) - X3的14脚,坡起开关状态获取
 *      有效的参数取值组合为(xNum, xPin)：(3, 4) - X3的4脚，挂车ABS功能开关状态获取; (1, 6) - X1的6脚,ASR和ESC功能开关状态获取; (1, 5) - X1的5脚,坡起开关状态获取
 * 返回值：0 - 关闭; 1 - 断开
 */
extern int8_t Xn_pin_StaGet(uint8_t xNum, uint8_t xPin);
/*PSW*/

/*form COM*/
extern void Com_vInit(void); //通讯初始化函数
extern void Com_vRun(void);  //通讯函数
extern void Dcm_vInit(void); //诊断初始化函数
extern void Dcm_vRun(void);  //诊断函数
extern void CANDbgMsgSend(uint32_t id, uint8_t* CANTxPacket);//CAN调试报文发送函数
extern void ABSWLampCtrl(uint8_t state);//ABS报警灯控制 0：off 1:on

//函数 InValActF() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void  InValActF(uint16_t period,uint16_t htime);//前桥进气阀动作函数 fact:0.1 unit: ms
//函数 OutValActF() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActF(uint16_t period, uint16_t htime);//前桥排气阀动作函数 fact:0.1 unit: ms
//函数 InValActR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是低电平，后是高电平
extern void  InValActR(uint16_t period,uint16_t htime);//后桥进气阀动作函数 fact:0.1 unit: ms
//函数 OutValActR() 参数 period 表示周期，htime 表示高电平时间，每个周期先是高电平，后是低电平
extern void OutValActR(uint16_t period, uint16_t htime);//后桥排气阀动作函数 fact:0.1 unit: ms

/* 函数:InLowSideSwF()
 * 描述: 前桥电磁进气阀低边开关控制。
 * 参数: 
 *      sw: 开关值 0 - 断开; 其他 - 闭合;
 */
void InLowSideSwF(uint32_t sw);

/* 函数: InLowSideStF()
 * 描述:  前桥电磁进气阀低边开关状态。
 * 参数: NONE
 * 返回值: 开关状态值 0 - 断开; 其他 - 闭合;
 */
uint32_t InLowSideStF(void);

/* 函数:InLowSideSwR()
 * 描述: 后桥电磁阀进气阀低边开关控制。
 * 参数: 
 *      sw: 开关值 0 - 断开; 其他 - 闭合;
 */
void InLowSideSwR(uint32_t sw);

/* 函数: InLowSideStR()
 * 描述:  后桥电磁进气阀低边开关状态。
 * 参数: NONE
 * 返回值: 开关状态值 0 - 断开; 其他 - 闭合;
 */
uint32_t InLowSideStR(void);

/* 函数: OutLowSideSwF()
 * 描述: 前桥电磁排气阀低边开关控制。
 * 参数: 
 *      sw: 开关值 0 - 断开; 其他 - 闭合;
 */
void OutLowSideSwF(uint8_t sw);

/* 函数: OutLowSideStF()
 * 描述: 前桥电磁排气阀低边开关状态。
 * 参数: NONE
 * 返回值: 开关状态值 0 - 断开; 其他 - 闭合;
 */
uint32_t OutLowSideStF(void);

/* 函数: OutLowSideSwR()
 * 描述: 后桥电磁排气阀低边开关控制。
 * 参数: 
 *      sw: 开关值 0 - 断开; 其他 - 闭合;
 */
void OutLowSideSwR(uint8_t sw);

/* 函数: OutLowSideStR()
 * 描述: 后桥电磁排气阀低边开关状态。
 * 参数: NONE
 * 返回值: 开关状态值 0 - 断开; 其他 - 闭合;
 */
uint32_t OutLowSideStR(void);
/*COM*/

/*form ASW*/
extern void RBSFunctionInit(void);  //RBS主功能模块功能初始化
extern void RBSFunctionCycle(void); //RBS主功能模块功能循环
/*ASW*/

/*============================================================================================
                                    All HEADER FILE INCLUDE
============================================================================================*/

#endif /* USER_RTE_H_ */

