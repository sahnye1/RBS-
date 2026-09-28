/**********************************************************************************************
 * 文 件 名: RTE.c
 * 作     者: LYJ
 * 功能描述: RBS 通讯、底层及应用层参数接口定义
 * 当前版本: V1.0.0
 * 修改历史:
 * 	  |历史版本      |日期           |修改人     |修改描述
 *        |V1.0.0        |2026-08-28     |刘彦军     |1、修改RTEfErrFlash/RTEfErrFlashOutBnd为RTEfErrEEprom/RTEfErrEEpromOutBnd
 *        |V1.0.0        |2026-09-05     |刘彦军     |1、COM层增加变量RTEfValDbgCtrl/RTEDbgPreF/RTEDbgPreR
 *                                                    2、将变量RTEMSGXBR从ASW层移至COM层
 *        |V1.0.0        |2026-09-07     |刘彦军     |1、删除ASR开/短路故障flag/增加21/22口开/短路故障flag
 *        |V1.0.0        |2026-09-10     |刘彦军     |1、增加前后桥开短路故障/前后桥配置控制变量RTEfFR_InOutValCfg
 *        |V1.0.0        |2026-09-11     |刘彦军     |1、封装X3_7/X3_10/X2_16/X4_16端相互组合配置前后桥用法
 *        |V1.0.0        |2026-09-11     |刘彦军     |1、修改实现配置配置不同波特率项目功能
 *        |V1.0.0        |2026-09-16     |刘彦军     |1、增加气压模块前后桥故障码,去除前后/附加桥故障码
 *                                                   |2、增加ESCM未标定/丢失/低/高电压故障
 *        |V1.0.0        |202-09-21      |刘彦军     |1、增加参数RTEYawRateDirecton/RTELatAccDirection/RTELongiDirection
 **********************************************************************************************/
#include "RTE.h"

/*============================================================================================
                                    GLOBAL VARIABLES
     @Description   This file contains variables that use for different modules.
============================================================================================*/
/*form PSW*/
uint16_t RTEvBat = 0;           //battery voltage fact:0.1 unit:v
uint16_t RTEvIgn = 0;           //ignition voltage fact:0.1 unit:v

uint16_t RTEBrkPreX4_14 = 0;    //X4_14 pin pressure fact:1 unit:kpa
uint16_t RTEBrkPreX4_15 = 0;    //X4_15 pin pressure fact:1 unit:kpa
uint8_t  RTEfErrPreX4_14 = 0;   //X4_14 pin pressure signal error flag  0:no error 1:error
uint8_t  RTEfErrPreX4_15 = 0;   //X4_15 pin pressure signal error flag  0:no error 1:error
uint16_t RTEWheelSpeedFL = 0;   //wheel speed FL fact:1 unit:mm/s
uint16_t RTEWheelSpeedFR = 0;   //wheel speed FR fact:1 unit:mm/s
uint16_t RTEWheelSpeedRL = 0;   //wheel speed RL fact:1 unit:mm/s
uint16_t RTEWheelSpeedRR = 0;   //wheel speed RR fact:1 unit:mm/s
uint16_t RTEWheelSpeedXL = 0;   //wheel speed XL fact:1 unit:mm/s
uint16_t RTEWheelSpeedXR = 0;   //wheel speed XR fact:1 unit:mm/s

uint8_t  RTETaskTime = 0;       //task execute time unit:ms fact:0.1
uint8_t  RTEfEESt = 0;          //EEPROM state flag 255:first write EEPROM 86:not first write EEPROM
uint8_t  RTEECUType = 0;        //ECU type 0:cab type 1:chassis type

struct PSWErr_Struct RTEfPSWErr;
/*PSW*/ 

/*form COM*/
uint16_t RTEYawRate = 0;                         //yaw rate fact: 0.01 unit:degree/s offset 30000 direction clockwise: >30000 direction anticlockwise: <30000
uint16_t RTELatAcc = 0;                          //lateral acceleration fact: 0.01 unit: m/s^2 offset 2100 direction left: >2100 dirction right: <2100
uint16_t RTELongiAcc = 0;                        //longitudinal acceleration fact: 0.01 unit: m/s^2 offset 2100 direction up: >2100 direction down: <2100

uint16_t RTEVehPitchAngle = 0;                   //vehicle pitch angle fact: 0.01 unit: degree
uint8_t  RTEVehAngleDir = 0;                     //vehicle pitch angle direction 0:elevation angle or level 1:depression angle

uint8_t  RTEYawRateDirecton = 0;                 //yaw rate direction 0:yaw clockwise or go straight 1:yaw anticlockwise
uint8_t  RTELatAccDirection = 0;                 //lateral acceleration direction 0:left low or level 1:right low
uint8_t  RTELongiDirection = 0;                  //longitudinal acceleration direction 0:head up or level 1:head down

uint8_t  RTEAccelPedal = 0;                      //accelerator pedal position 0 to 100%
uint16_t RTEEngineSpeed = 0;                     //engine speed fact 0.125 unit:rpm
uint8_t  RTEGearPosition = 0;                    //gear position ETC2 value
uint8_t  RTEGearSelectedPosition = 0;            //gear selected position ETC2 value
uint8_t  RTEKeyPosition = 0;                     //key position 0:OFF 1:ACC/ON/START
uint16_t RTEGasCylinderPreF = 0;                 //front axle gas cylinder pressure fact:1 unit:kpa. if no gas cylinder pressure,set to 0XFF
uint16_t RTEGasCylinderPreR = 0;                 //rear axle gas cylinder pressure fact:1 unit:kpa. if no gas cylinder pressure,set to 0XFF
uint8_t  RTETransReadyForBrakeRelease = 0;       //transmission ready for brake release from CAN message ETC7 0:not ready 1:ready
uint16_t RTEExVehicleSpeed = 0;                  //external vehicle speed fact:0.01 unit:km/h
uint8_t  RTEParkBrkSt = 0;                       //park brake state 0:park brake not active 1:park brake active
uint16_t RTEVSWAngel = 0;                        //steering wheel angel fact: 1/1024 unit: rad offset:-31.374rad
uint8_t  RTEDriversDemandEngPercentTorque = 0;   //Drivers Demand Engine Torque. UNIT:%
uint8_t  RTEActualEngPercentTorque = 0;          //Actual Engine Torque. UNIT:%
uint8_t  RTEComHaltBrakeSwitch = 0;              //communication layer halt brake switch state signal from CAN. 0:off 1:on
uint8_t  RTEComABSOffRoadSwitch = 0;             //ABS off road switch state signal from CAN. 0:off 1:on
uint8_t  RTEComESCOffSwitch = 0;                 //ESC off switch state signal from CAN. 0:off 1:on
uint8_t  RTEComHSASwitch = 0;                    //HSA switch state signal from CAN. 0:off 1:on
uint8_t  RTEComASRSwitch = 0;                    //ASR switch state signal from CAN. 0:off 1:on

struct COMErr_Struct RTEfCOMErr;

uint8_t  RTEfSysUseSt = 0;                       //system use state 0:normal state 1:FCT test 2:OEM EOL test
uint8_t  RTEReqClearDTC = 0;                     //Clear All DTC(erase flash)0:no request 1:request
uint8_t  RTEfConfig = 0;                         //configure vehicle parameters flag 0:no configure 1:configure
uint8_t  RTEfConfig2Def = 0;                     //configure vehicle parameters to default flag 0:no configure 1:configure
uint8_t  RTEfBin2EE = 0;                         //configure bin file parameters to eeprom 0:no configure 1:configure
uint8_t  RTEfClearCVW = 0;                       //clear calculate vehicle weight flag
uint8_t  RTEfBoot;                               //Boot Start
//****************************************************************************
//RTEVMode:
//MODE_4S4M	    0
//MODE_6S6M	    1
//****************************************************************************
uint8_t RTEVMode = 0;                            //vehicle mode r/w

//****************************************************************************
//RTETrMode:
//TRAILER_NONE 	0
//TRAILER    	1
//****************************************************************************
uint8_t RTETrMode = 0;                           //trailer mode r/w

uint8_t RTEfDriveAxle = 0;                       //0:rear axle 1:additional axle 2:front axle
uint8_t RTEPSPosNumber = 0;                      //pressure sensor position and number 0:1rear 1:1front 2:two sensor
uint8_t RTEfSASPlugDirection = 0;                //r/w 0:down 1:up
uint8_t RTEfESCCPlugDirection = 0;               //ABSESC Plug Direction 0:front 1:back
uint8_t RTEABSOFFSWST = 0;                       //ABS Off Switch State r/w 0:function on 1:function off
uint8_t RTEASROFFSWST = 0;                       //ASR Off Switch State r/w 0:function on 1:function off
uint8_t RTEHSAONSWST = 0;                        //HSA On Switch State r/w 0:function off 1:function on
uint8_t RTEESCOFFSWST = 0;                       //ESC Switch State r/w 0:function on 1:function off
uint8_t RTEHALTBRAKESWST = 0;                    //Halt Brake Switch State r/w 0:function off 1:function on

uint8_t RTEfABSEn = 0;                           //ABS Function Enable Flag r/w 0:not enable, 1:enable
uint8_t RTEfASREn = 0;                           //ASR Function Enable Flag r/w 0:not enable, 1:enable
uint8_t RTEfHSAEn = 0;                           //HSA Function Enable Flag r/w 0:not enable, 1:enable
uint8_t RTEfESCEn = 0;                           //ESC Function Enable Flag r/w 0:not enable, 1:enable
uint8_t RTEfHaltBrakeEn = 0;                     //Halt Brake Function Enable Flag r/w 0:not enable, 1:enable
uint8_t RTEfXBREn = 0;                           //XBR Function Enable Flag r/w 0:not enable, 1:enable

uint16_t RTEVBase[4] = { 0 };                    //vehicle base r/w 1:1mm
uint8_t  RTEVWSToothCnt = 0;                     //vehicle wheel speed sensor tooth counter  r/w
uint16_t RTEVWRDiameter = 0;                     //vehicle wheel roll diameter r/w 1:1mm
uint8_t  RTEMaxSpeed = 0;                        //vehicle max speed r/w 1:1km/h
uint8_t  RTESteerRatio = 0;                      //Steering ratio 0:20.5;1:23;2:22.2-26.22;3:17-20;4:17.6-20.8;5:18.3
uint8_t  RTEEngTqLmtTest = 0;                    //engine torque limit test.range:-125 to 125% fact:1%/bit offset:-125% 255:invalid
uint16_t RTEBrkPreF = 0;                         //front axle pressure fact:1 unit:kpa.
uint16_t RTEBrkPreR = 0;                         //rear axle pressure fact:1 unit:kpa.
uint8_t  RTEfErrPreF;                            //front axle pressure signal error flag  0:no error 1:error
uint8_t  RTEfErrPreR;                            //rear axle pressure signal error flag  0:no error 1:error
uint8_t  RTEXBRDbgLadF = 0;                      //XBR debug ladder, front bridge ,0-50
uint8_t  RTEXBRDbgLadR = 0;                      //XBR debug ladder, rear bridge ,0-50
uint8_t  RTEXBRDbgLadX = 0;                      //XBR debug ladder, additional bridge ,0-50
uint8_t  RTEXBRDbgLadTr = 0;                     //XBR debug ladder, trailer ,0-50

uint8_t  RTEfTestMode = 0;                       //test mode 0:normal mode 1:factory test mode 2:OEM test mode
uint8_t  RTEfAddAxlePos = 0;                     //additional axle position used in MODE_6S6M 0:middle 1:rear

uint8_t  RTEfValDbgCtrl = 0;                     //valve debug control flag 0:not control 1:control
uint16_t RTEDbgPreF = 0;                         //debug front pressure request fact:1 unit:kpa
uint16_t RTEDbgPreR = 0;                         //debug front pressure request fact:1 unit:kpa

// 0: RTEBrkPreF <-> RTEBrkPreX4_14, RTEBrkPreR <-> RTEBrkPreX4_14, RTEfErrPreF <-> RTEfErrPreX4_14
// 1: RTEBrkPreF <-> RTEBrkPreX4_14, RTEBrkPreR <-> RTEBrkPreX4_14, RTEfErrPreR <-> RTEfErrPreX4_14
// 2: RTEBrkPreF <-> RTEBrkPreX4_15, RTEBrkPreR <-> RTEBrkPreX4_15, RTEfErrPreF <-> RTEfErrPreX4_15
// 3: RTEBrkPreF <-> RTEBrkPreX4_15, RTEBrkPreR <-> RTEBrkPreX4_15, RTEfErrPreR <-> RTEfErrPreX4_15
// 4: RTEBrkPreF <-> RTEBrkPreX4_14, RTEBrkPreR <-> RTEBrkPreX4_15, RTEfErrPreF <-> RTEfErrPreX4_14, RTEfErrPreR <-> RTEfErrPreX4_15
// 5: RTEBrkPreF <-> RTEBrkPreX4_15, RTEBrkPreR <-> RTEBrkPreX4_14, RTEfErrPreF <-> RTEfErrPreX4_15, RTEfErrPreR <-> RTEfErrPreX4_14
uint8_t RTEfPressCfg;

//0: RTEfInValveF<->RTEfInValveX3_7   RTEfOutValveF<->RTEfOutValveX3_10  RTEfInValveR<->RTEfInValveX4_16  RTEfOutValveR<->RTEfOutValveX2_16
//1: RTEfInValveF<->RTEfInValveX4_16  RTEfOutValveF<->RTEfOutValveX2_16  RTEfInValveR<->RTEfInValveX3_7   RTEfOutValveR<->RTEfOutValveX3_10
//2: RTEfInValveF<->RTEfInValveX3_7   RTEfOutValveF<->RTEfOutValveX2_16  RTEfInValveR<->RTEfInValveX4_16  RTEfOutValveR<->RTEfOutValveX3_10
//3: RTEfInValveF<->RTEfInValveX4_16  RTEfOutValveF<->RTEfOutValveX3_10  RTEfInValveR<->RTEfInValveX3_7   RTEfOutValveR<->RTEfOutValveX2_16
uint8_t RTEfFR_InOutValCfg;

uint8_t  RTEfABSOffSwitchSrc = 0;
uint8_t  RTEfASROffSwitchSrc = 0;
uint8_t  RTEfESCOffSwitchSrc = 0;
uint8_t  RTEfHSASwitchSrc = 0;

uint8_t  RTEfABSOffSwitchType = 0;
uint8_t  RTEfASROffSwitchType = 0;
uint8_t  RTEfESCOffSwitchType = 0;
uint8_t  RTEfHSASwitchType = 0;

uint8_t  RTEABSLPCTRLSrc;          //0:X1_02/X3_1  1:X1_13  2:X1_15  254: CAN, 255: NONE
uint8_t  RTEESCLPCTRLSrc;          //0:X1_02/X3_1  1:X1_13  2:X1_15  254: CAN, 255: NONE
uint8_t  RTEHSALPCTRLSrc;          //0:X1_02/X3_1  1:X1_13  2:X1_15  254: CAN, 255: NONE
uint8_t  RTEASRLPCTRLSrc;          //0:X1_02/X3_1  1:X1_13  2:X1_15  254: CAN, 255: NONE

uint16_t RTEGroundTest =0;         //Ground test

// UDS
uint8_t  RTEEOLT = 0;

uint8_t DATA_0xF180[7] = {0};
uint8_t DATA_0xF181[7] = {0};
uint8_t DATA_0xF182[7] = {0};
uint8_t DATA_0xF183[42] = {0};
uint8_t DATA_0xF184[42] = {0};
uint8_t DATA_0xF185[42] = {0};
uint8_t DATA_0xF187[13] = {0};
uint8_t DATA_0xF18A[8] = {0};
uint8_t DATA_0xF18B[4] = {0};
uint8_t DATA_0xF18C[12] = {0};
uint8_t DATA_0xF190[17] = {0};
uint8_t DATA_0xF193[6] = {0};
uint8_t DATA_0xF195[6] = {0};
uint8_t DATA_0xF197[9] = {0};
uint8_t DATA_0xF192[5] = {0};
uint8_t DATA_0xF194[7] = {0};

/*configure parameters*/
uint8_t DATA_0x5010[8] = {0};                    //base
uint8_t DATA_0x5011[1] = {0};                    //tooth counter
uint8_t DATA_0x5012[2] = {0};                    //diameter
uint8_t DATA_0x5013[2] = {0};                    //vehicle mode
uint8_t DATA_0x5014[2] = {0};                    //max speed
uint8_t DATA_0x5015[1] = {0};                    //ESC Switch Type
uint8_t DATA_0x5016[1] = {0};                    //reserved
uint8_t DATA_0x5017[1] = {0};                    //ABS Function Switch 0:not enable, 1:enable
uint8_t DATA_0x5018[1] = {0};                    //ESC Function Switch 0:not enable, 1:enable
uint8_t DATA_0x5019[1] = {0};                    //ASR Function Switch 0:not enable, 1:enable
uint8_t DATA_0x501A[1] = {0};                    //HSA Function Switch 0:not enable, 1:enable
uint8_t DATA_0x501B[1] = {0};                    //XBR Function Switch 0:not enable, 1:enable
uint8_t DATA_0x501C[1] = {0};                    //Steering ratio 0:20.5;1:23;2:22.2-26.22;3:17-20;4:17.6-20.8;5:18.3
uint8_t DATA_0x501D[1] = {0};                    //Communication Baud Rate 0:250k 1:500k
uint8_t DATA_0x501E[1] = {0};                    //SAS Type
uint8_t DATA_0x501F[1] = {0};                    //ESC Type
uint8_t DATA_0x5678[1] = {0};                    //configure parameters to default values
uint8_t DATA_0x5200[154] = {0};                  // configure parameters when change the ECU
uint8_t DATA_0x5201[150] = {0};                  // configure calibration parameters
BINPARA BINPARA0;
_XBR  RTEMSGXBR;
/*COM*/

/*form ASW*/
uint8_t RTEDTC[224]=
{
	0X1B,0X03,0X05,0X00,//左前ABS阀故障               //RTEDTC[3]  1 (DTCCNT2EE[0])&0x0F
	0X1C,0X03,0X05,0X00,//右前ABS阀故障               //RTEDTC[7]  2 (DTCCNT2EE[0]>>4)&0x0F
	0X1D,0X03,0X05,0X00,//左后ABS阀故障               //RTEDTC[11] 3 (DTCCNT2EE[1])&0x0F
	0X1E,0X03,0X05,0X00,//右后ABS阀故障               //RTEDTC[15] 4 (DTCCNT2EE[1]>>4)&0x0F
	0X6D,0XF0,0XE5,0X00,//气压调节模块前桥电磁阀故障  //RTEDTC[19] 5 (DTCCNT2EE[2])&0x0F

	0X15,0X03,0X05,0X00,//左前轮速传感器开路或短路故障//RTEDTC[23] 6 (DTCCNT2EE[2]>>4)&0x0F
	0X15,0X03,0X01,0X00,//左前轮速传感器间隙过大故障  //RTEDTC[27] 7 (DTCCNT2EE[3])&0x0F
	0X15,0X03,0X0A,0X00,//左前轮速传感器信号不稳故障  //RTEDTC[31] 8 (DTCCNT2EE[3]>>4)&0x0F
        
	0X16,0X03,0X05,0X00,//右前轮速传感器开路或短路故障//RTEDTC[35] 9 (DTCCNT2EE[4])&0x0F
	0X16,0X03,0X01,0X00,//右前轮速传感器间隙过大故障  //RTEDTC[39] 10 (DTCCNT2EE[4]>>4)&0x0F
	0X16,0X03,0X0A,0X00,//右前轮速传感器信号不稳故障  //RTEDTC[43] 11 (DTCCNT2EE[5])&0x0F
        
	0X17,0X03,0X05,0X00,//左后轮速传感器开路或短路故障//RTEDTC[47] 12 (DTCCNT2EE[5]>>4)&0x0F
	0X17,0X03,0X01,0X00,//左后轮速传感器间隙过大故障  //RTEDTC[51] 13 (DTCCNT2EE[6])&0x0F
	0X17,0X03,0X0A,0X00,//左后轮速传感器信号不稳故障  //RTEDTC[55] 14 (DTCCNT2EE[6]>>4)&0x0F
        
	0X18,0X03,0X05,0X00,//右后轮速传感器开路或短路故障//RTEDTC[59] 15 (DTCCNT2EE[7])&0x0F
	0X18,0X03,0X01,0X00,//右后轮速传感器间隙过大故障  //RTEDTC[63] 16 (DTCCNT2EE[7]>>4)&0x0F
	0X18,0X03,0X0A,0X00,//右后轮速传感器信号不稳故障  //RTEDTC[67] 17 (DTCCNT2EE[8])&0x0F
        
	0X76,0X02,0X0D,0X00,//轮速故障                    //RTEDTC[71] 18 (DTCCNT2EE[8]>>4)&0x0F
	0X76,0X02,0X0C,0X00,//超速故障                    //RTEDTC[75] 19 (DTCCNT2EE[9])&0x0F
        
	/*上电瞬间只检测过压故障，上电后检测低压故障和过压故障
	 * 上电后欠压1秒后，且车速大于7.2km/h可以报欠压故障，
	 * 上电后过压400毫秒后可以报过压故障
	 * */
	0X23,0X03,0X04,0X00,//低压故障                   //RTEDTC[79] 20 (DTCCNT2EE[9]>>4)&0x0F

	0X73,0X02,0X03,0X00,//高压故障                   //RTEDTC[83] 21 (DTCCNT2EE[10])&0x0F
	0X75,0X02,0X0C,0X00,//EEPROM故障                 //RTEDTC[87] 22 (DTCCNT2EE[10]>>4)&0x0F
	0X22,0X03,0X07,0X00,//阀电源（继电器）故障       //RTEDTC[91] 23 (DTCCNT2EE[11])&0x0F
	0X22,0X03,0X05,0X00,//驱动芯片故障               //RTEDTC[95] 24 (DTCCNT2EE[11]>>4)&0x0F
	0X26,0X03,0X05,0X00,//气压调节模块后桥电磁阀故障 //RTEDTC[99] 25 (DTCCNT2EE[12])&0x0F

	0X27,0X03,0X05,0X00,//预留1                      //RTEDTC[103] 26 (DTCCNT2EE[12]>>4)&0x0F
	0X28,0X03,0X05,0X00,//预留2                      //RTEDTC[107] 27 (DTCCNT2EE[13])&0x0F
	0X15,0X04,0X05,0X00,//前桥PRS故障                //RTEDTC[111] 28 (DTCCNT2EE[13]>>4)&0x0F
	0X16,0X04,0X05,0X00,//后桥PRS故障                //RTEDTC[115] 29 (DTCCNT2EE[14])&0x0F
	0X12,0XF0,0XED,0X00,//ESC未标定故障              //RTEDTC[119] 30 (DTCCNT2EE[14]>>4)&0x0F

	0X12,0XF0,0XEC,0X00,//ESC系统故障                //RTEDTC[123] 31 (DTCCNT2EE[15])&0x0F
	0X0F,0X07,0X0D,0X00,//SAS未标定故障              //RTEDTC[127] 32 (DTCCNT2EE[15]>>4)&0x0F
	0X0F,0X07,0X0C,0X00,//SAS系统故障                //RTEDTC[131] 33 (DTCCNT2EE[16])&0x0F
	/*6S6M*/
	0X1F,0X03,0X05,0X00,//附加桥左ABS阀故障          //RTEDTC[135] 34 (DTCCNT2EE[16]>>4)&0x0F
	0X20,0X03,0X05,0X00,//附加桥右ABS阀故障          //RTEDTC[139] 35 (DTCCNT2EE[17])&0x0F

	0X19,0X03,0X05,0X00,//附加桥左轮速传感器开路或短路故障//RTEDTC[143] 36 (DTCCNT2EE[17]>>4)&0x0F
	0X19,0X03,0X01,0X00,//附加桥左轮速传感器间隙过大故障  //RTEDTC[147] 37 (DTCCNT2EE[18])&0x0F
	0X19,0X03,0X0A,0X00,//附加桥左轮速传感器信号不稳故障  //RTEDTC[151] 38 (DTCCNT2EE[18]>>4)&0x0F

	0X1A,0X03,0X05,0X00,//附加桥右轮速传感器开路或短路故障//RTEDTC[155] 39 (DTCCNT2EE[19])&0x0F
	0X1A,0X03,0X01,0X00,//附加桥右轮速传感器间隙过大故障  //RTEDTC[159] 40 (DTCCNT2EE[19]>>4)&0x0F
	0X1A,0X03,0X0A,0X00,//附加桥右轮速传感器信号不稳故障  //RTEDTC[163] 41 (DTCCNT2EE[20])&0x0F

	0X7F,0X02,0X05,0X00,//私有CAN故障                     //RTEDTC[167] 42 (DTCCNT2EE[20]>>4)&0x0F
	0X12,0XF0,0XE3,0X00,//姿态传感器高电压故障            //RTEDTC[171] 43 (DTCCNT2EE[21])&0x0F
	0X12,0XF0,0XE4,0X00,//姿态传感器低电压故障            //RTEDTC[175] 44 (DTCCNT2EE[21]>>4)
	0X0F,0X07,0X03,0X00,//方向盘转角传感器高电压故障      //RTEDTC[179] 45 (DTCCNT2EE[22])&0x0F
	0X0F,0X07,0X04,0X00,//方向盘转角传感器低电压故障      //RTEDTC[183] 46 (DTCCNT2EE[22]>>4)

	0X12,0XF0,0XEA,0X00,                                  //姿态传感器信号异常故障(陕汽不支持此故障，由姿态传感器内部故障代替) //RTEDTC[187] 47 (DTCCNT2EE[23])&0x0F

	0X75,0X02,0X02,0X00,//配置参数未写入故障              //RTEDTC[191] 48 (DTCCNT2EE[23]>>4)
	0XB1,0X04,0X09,0X00,//ASR功能相关报文接收超时故障     //RTEDTC[195] 49 (DTCCNT2EE[24])&0x0F
	0XB2,0X04,0X09,0X00,//ESC功能相关报文接收超时故障     //RTEDTC[199] 50 (DTCCNT2EE[24]>>4)
	0XB3,0X04,0X09,0X00,//HSA功能相关报文接收超时故障     //RTEDTC[203] 51 (DTCCNT2EE[25])&0x0F
	0XB4,0X04,0X09,0X00,//XBR功能相关报文接收超时故障     //RTEDTC[207] 52 (DTCCNT2EE[25]>>4)
	0X00,0X00,0X00,0X00,//预留3                           //RTEDTC[211] 53 (DTCCNT2EE[26])&0x0F
	0X00,0X00,0X00,0X00,//预留4                           //RTEDTC[215] 54 (DTCCNT2EE[26]>>4)
	0X00,0X00,0X00,0X00,//预留5                           //RTEDTC[219] 55 (DTCCNT2EE[27])&0x0F
	0X00,0X00,0X00,0X00,//预留6                           //RTEDTC[223] 56 (DTCCNT2EE[27]>>4)
};

uint8_t RTEfCalib;           //calibration bin file parameters to ram 0:no configure 1:configure
uint8_t RTEfValWssTestForbit;

uint8_t RTEfCAN0TxForbid;
uint8_t RTEfCAN0RxForbid;
uint8_t RTEComBaudRate;      //Communication Baud Rate 0:250k 1:500k
uint8_t RTEfDbgMsgSW, RTEfDbgMsgSW1, RTEfDbgMsgSW2, RTEfDbgMsgSW3;

_EBC1 RTEMSGEBC1;
_EBC2 RTEMSGEBC2;
_VDC1 RTEMSGVDC1;
_VDC2 RTEMSGVDC2;
_TSC1 RTEMSGTSC1_E;
_TSC1 RTEMSGTSC1_ER;
_TSC1 RTEMSGTSC1_AR;
_TSC1 RTEMSGTSC1_EXR;
_EBC5 RTEMSGEBC5;
_HRW  RTEMSGHRW;
/*ASW*/
