/**
 * @file    psw_data.c
 * @brief   PSW 中间变量层 — 驱动 → PSW* → RTE* 数据转发
 *
 * @details PSWData_Refresh(): 每 10ms 读各驱动内部状态 → 写 PSW*。
 *          PSWDataToRte():     PSW* 经校验/掩蔽 → 写 RTE*。
 */

#include "psw_data.h"
#include "RTE.h"
#include "bts724g.h"
#include "adc.h"
#include "wheel_speed.h"

/* ========================================================================== */
/*  常量                                                                       */
/* ========================================================================== */


#define DATA_INVALID          0xFFFFu  /* 无效数据标记 */
#define VEHICLE_MODE_4S4M     0u       /* 4S4M 模式 */

/* ========================================================================== */
/*  PSW* 中间变量定义                                                          */
/* ========================================================================== */

/* ---- 核心数值 ---- */
uint8_t  PSWTaskTime      = 0u;
uint16_t PSWvIgn          = 0u;

/* ---- 压力传感器 ---- */
uint16_t PSWBrkPreX2_7    = 0u;
uint16_t PSWBrkPreX3_10   = 0u;
uint8_t  PSWfErrPreX2_7   = 0u;
uint8_t  PSWfErrPreX3_10  = 0u;


/* ---- 轮速 ---- */
uint16_t PSWWheelSpeedFL  = 0u;
uint16_t PSWWheelSpeedFR  = 0u;
uint16_t PSWWheelSpeedRL  = 0u;
uint16_t PSWWheelSpeedRR  = 0u;
uint16_t PSWWheelSpeedXL  = 0u;
uint16_t PSWWheelSpeedXR  = 0u;

/* ---- 轮速传感器故障 ---- */
uint8_t  PSWErrWssOpenFL  = 0u; uint8_t PSWErrWssShortFL = 0u; uint8_t PSWErrWssGapFL = 0u;
uint8_t  PSWErrWssOpenFR  = 0u; uint8_t PSWErrWssShortFR = 0u; uint8_t PSWErrWssGapFR = 0u;
uint8_t  PSWErrWssOpenRL  = 0u; uint8_t PSWErrWssShortRL = 0u; uint8_t PSWErrWssGapRL = 0u;
uint8_t  PSWErrWssOpenRR  = 0u; uint8_t PSWErrWssShortRR = 0u; uint8_t PSWErrWssGapRR = 0u;
uint8_t  PSWErrWssOpenXL  = 0u; uint8_t PSWErrWssShortXL = 0u; uint8_t PSWErrWssGapXL = 0u;
uint8_t  PSWErrWssOpenXR  = 0u; uint8_t PSWErrWssShortXR = 0u; uint8_t PSWErrWssGapXR = 0u;

/* ---- ABS 阀故障 ---- */
uint8_t  PSWErrInValOpenFL  = 0u;  uint8_t PSWErrInValShortFL  = 0u;
uint8_t  PSWErrOutValOpenFL = 0u;  uint8_t PSWErrOutValShortFL = 0u;
uint8_t  PSWErrInValOpenFR  = 0u;  uint8_t PSWErrInValShortFR  = 0u;
uint8_t  PSWErrOutValOpenFR = 0u;  uint8_t PSWErrOutValShortFR = 0u;
uint8_t  PSWErrInValOpenRL  = 0u;  uint8_t PSWErrInValShortRL  = 0u;
uint8_t  PSWErrOutValOpenRL = 0u;  uint8_t PSWErrOutValShortRL = 0u;
uint8_t  PSWErrInValOpenRR  = 0u;  uint8_t PSWErrInValShortRR  = 0u;
uint8_t  PSWErrOutValOpenRR = 0u;  uint8_t PSWErrOutValShortRR = 0u;
uint8_t  PSWErrInValOpenXL  = 0u;  uint8_t PSWErrInValShortXL  = 0u;
uint8_t  PSWErrOutValOpenXL = 0u;  uint8_t PSWErrOutValShortXL = 0u;
uint8_t  PSWErrInValOpenXR  = 0u;  uint8_t PSWErrInValShortXR  = 0u;
uint8_t  PSWErrOutValOpenXR = 0u;  uint8_t PSWErrOutValShortXR = 0u;
/* ---- 21口 阀故障 (原挂车 Tr 阀) ---- */
uint8_t  PSWErr21InValveOpen   = 0u; uint8_t PSWErr21InValveShort   = 0u;
uint8_t  PSWErr21OutValveOpen  = 0u; uint8_t PSWErr21OutValveShort  = 0u;

/* ---- 22口 阀故障 (原 ASR F/R 阀) ---- */
uint8_t  PSWErr22InValveOpen   = 0u; uint8_t PSWErr22InValveShort   = 0u;
uint8_t  PSWErr22OutValveOpen  = 0u; uint8_t PSWErr22OutValveShort  = 0u;

/* ---- TR_ASR (新 RTE 无对应字段, 仅 PSW 内部/调试打印) ---- */
uint8_t  PSWErrASRValveOpenX  = 0u; uint8_t PSWErrASRValveShortX = 0u;

/* ---- 驱动芯片故障 ---- */
uint8_t  PSWErrOpenDrive724_U6  = 0u; uint8_t PSWErrShortDrive724_U6  = 0u;
uint8_t  PSWErrOpenDrive724_U9  = 0u; uint8_t PSWErrShortDrive724_U9  = 0u;
uint8_t  PSWErrOpenDrive724_U12 = 0u; uint8_t PSWErrShortDrive724_U12 = 0u;
uint8_t  PSWErrOpenDrive724_U13 = 0u; uint8_t PSWErrShortDrive724_U13 = 0u;
uint8_t  PSWErrOpenDrive724_U19 = 0u; uint8_t PSWErrShortDrive724_U19 = 0u;
uint8_t  PSWErrDriveChip        = 0u;

/* ---- VPOWER 电源故障 (仅 VPOWER 一路, 无独立电池检测) ---- */
uint8_t  PSWErrVpwrLo       = 0u; uint8_t PSWErrVpwrHi  = 0u;
uint8_t  PSWErrRelay         = 0u;
/* ---- EEPROM ---- */
uint8_t  PSWfErrEEprom       = 0u;
uint8_t  PSWfErrEEpromOutBnd = 0u;


uint32_t resetReason;     //模块重启的原因

/* ========================================================================== */
/*  PSWData_Refresh() — 从各驱动读取内部状态, 填充 PSW* 中间变量                 */
/* ========================================================================== */
void PSWData_Refresh(void)
{
    /* ---- 1. VPOWER 电压 (adc, P7.2 AN10, 继电器后) ---- */
    PSWvIgn        = Vpower_GetVoltage();   /* 板载仅一路 VPOWER, 无独立电池检测 */
    PSWErrVpwrHi   = Vpower_IsHighAlarm() ? 1u : 0u;
    PSWErrVpwrLo   = Vpower_IsLowAlarm()  ? 1u : 0u;

    /* ---- 2. 压力传感器 (adc) ---- */
    PSWBrkPreX2_7  = Pressure_GetFrontKpa();
    PSWBrkPreX3_10 = Pressure_GetRearKpa();
    PSWfErrPreX2_7 = Pressure_IsFrontFault() ? 1u : 0u;
    PSWfErrPreX3_10 = Pressure_IsRearFault()  ? 1u : 0u;

    /* ---- 3. 轮速传感器故障 (adc) ---- */
    /* 通道映射 (按硬件引脚): CH0=P6.4-FL, CH1=P6.5-LR, CH2=P7.0-RR,
     *                       CH3=P7.1-RF, CH4=P7.3-LX, CH5=P7.4-RX        */
    PSWErrWssOpenFL  = g_sensor_fault_status.ch0_fault_open          ? 1u : 0u;  /* CH0 P6.4 FL */
    PSWErrWssShortFL = g_sensor_fault_status.ch0_fault_short         ? 1u : 0u;
    PSWErrWssGapFL   = g_sensor_fault_status.ch0_fault_gap_too_large ? 1u : 0u;

    PSWErrWssOpenRL  = g_sensor_fault_status.ch1_fault_open          ? 1u : 0u;  /* CH1 P6.5 LR */
    PSWErrWssShortRL = g_sensor_fault_status.ch1_fault_short         ? 1u : 0u;
    PSWErrWssGapRL   = g_sensor_fault_status.ch1_fault_gap_too_large ? 1u : 0u;

    PSWErrWssOpenRR  = g_sensor_fault_status.ch2_fault_open          ? 1u : 0u;  /* CH2 P7.0 RR */
    PSWErrWssShortRR = g_sensor_fault_status.ch2_fault_short         ? 1u : 0u;
    PSWErrWssGapRR   = g_sensor_fault_status.ch2_fault_gap_too_large ? 1u : 0u;

    PSWErrWssOpenFR  = g_sensor_fault_status.ch3_fault_open          ? 1u : 0u;  /* CH3 P7.1 RF */
    PSWErrWssShortFR = g_sensor_fault_status.ch3_fault_short         ? 1u : 0u;
    PSWErrWssGapFR   = g_sensor_fault_status.ch3_fault_gap_too_large ? 1u : 0u;

    PSWErrWssOpenXL  = g_sensor_fault_status.ch4_fault_open          ? 1u : 0u;  /* CH4 P7.3 LX */
    PSWErrWssShortXL = g_sensor_fault_status.ch4_fault_short         ? 1u : 0u;
    PSWErrWssGapXL   = g_sensor_fault_status.ch4_fault_gap_too_large ? 1u : 0u;

    PSWErrWssOpenXR  = g_sensor_fault_status.ch5_fault_open          ? 1u : 0u;  /* CH5 P7.4 RX */
    PSWErrWssShortXR = g_sensor_fault_status.ch5_fault_short         ? 1u : 0u;
    PSWErrWssGapXR   = g_sensor_fault_status.ch5_fault_gap_too_large ? 1u : 0u;

    /* ---- 4. 轮速 (wheel_speed, mm/s) ---- */
    PSWWheelSpeedFL = g_wheel_speed_rpm[1];   /* CH1 P12.0 */
    PSWWheelSpeedFR = g_wheel_speed_rpm[5];   /* CH5 P12.4 */
    PSWWheelSpeedRL = g_wheel_speed_rpm[4];   /* CH4 P12.3 */
    PSWWheelSpeedRR = g_wheel_speed_rpm[3];   /* CH3 P12.2 */
    PSWWheelSpeedXL = g_wheel_speed_rpm[0];   /* CH0 P8.2 */
    PSWWheelSpeedXR = g_wheel_speed_rpm[2];   /* CH2 P8.1 */

    /* ---- 5. 阀故障 (bts724g) ---- */
    PSWErrInValOpenFL   = g_bts724g_fault_status.bts724g_fault_open[VALVE_LFI]     ? 1u : 0u;
    PSWErrInValShortFL  = g_bts724g_fault_status.bts724g_fault_short[VALVE_LFI]    ? 1u : 0u;
    PSWErrOutValOpenFL  = g_bts724g_fault_status.bts724g_fault_open[VALVE_LFO]     ? 1u : 0u;
    PSWErrOutValShortFL = g_bts724g_fault_status.bts724g_fault_short[VALVE_LFO]    ? 1u : 0u;

    PSWErrInValOpenFR   = g_bts724g_fault_status.bts724g_fault_open[VALVE_RFI]     ? 1u : 0u;
    PSWErrInValShortFR  = g_bts724g_fault_status.bts724g_fault_short[VALVE_RFI]    ? 1u : 0u;
    PSWErrOutValOpenFR  = g_bts724g_fault_status.bts724g_fault_open[VALVE_RFO]     ? 1u : 0u;
    PSWErrOutValShortFR = g_bts724g_fault_status.bts724g_fault_short[VALVE_RFO]    ? 1u : 0u;

    PSWErrInValOpenRL   = g_bts724g_fault_status.bts724g_fault_open[VALVE_LRI]     ? 1u : 0u;
    PSWErrInValShortRL  = g_bts724g_fault_status.bts724g_fault_short[VALVE_LRI]    ? 1u : 0u;
    PSWErrOutValOpenRL  = g_bts724g_fault_status.bts724g_fault_open[VALVE_LRO]     ? 1u : 0u;
    PSWErrOutValShortRL = g_bts724g_fault_status.bts724g_fault_short[VALVE_LRO]    ? 1u : 0u;

    PSWErrInValOpenRR   = g_bts724g_fault_status.bts724g_fault_open[VALVE_RRI]     ? 1u : 0u;
    PSWErrInValShortRR  = g_bts724g_fault_status.bts724g_fault_short[VALVE_RRI]    ? 1u : 0u;
    PSWErrOutValOpenRR  = g_bts724g_fault_status.bts724g_fault_open[VALVE_RRO]     ? 1u : 0u;
    PSWErrOutValShortRR = g_bts724g_fault_status.bts724g_fault_short[VALVE_RRO]    ? 1u : 0u;

    PSWErrInValOpenXL   = g_bts724g_fault_status.bts724g_fault_open[VALVE_LXI]     ? 1u : 0u;
    PSWErrInValShortXL  = g_bts724g_fault_status.bts724g_fault_short[VALVE_LXI]    ? 1u : 0u;
    PSWErrOutValOpenXL  = g_bts724g_fault_status.bts724g_fault_open[VALVE_LXO]     ? 1u : 0u;
    PSWErrOutValShortXL = g_bts724g_fault_status.bts724g_fault_short[VALVE_LXO]    ? 1u : 0u;

    PSWErrInValOpenXR   = g_bts724g_fault_status.bts724g_fault_open[VALVE_RXI]     ? 1u : 0u;
    PSWErrInValShortXR  = g_bts724g_fault_status.bts724g_fault_short[VALVE_RXI]    ? 1u : 0u;
    PSWErrOutValOpenXR  = g_bts724g_fault_status.bts724g_fault_open[VALVE_RXO]     ? 1u : 0u;
    PSWErrOutValShortXR = g_bts724g_fault_status.bts724g_fault_short[VALVE_RXO]    ? 1u : 0u;

    /* ---- 5. 21口 / 22口 阀故障 ---- */
    PSWErr21InValveOpen   = g_bts724g_fault_status.bts724g_fault_open[VALVE_21IN]   ? 1u : 0u;
    PSWErr21InValveShort  = g_bts724g_fault_status.bts724g_fault_short[VALVE_21IN]  ? 1u : 0u;
    PSWErr21OutValveOpen  = g_bts724g_fault_status.bts724g_fault_open[VALVE_21OUT]  ? 1u : 0u;
    PSWErr21OutValveShort = g_bts724g_fault_status.bts724g_fault_short[VALVE_21OUT] ? 1u : 0u;

    PSWErr22InValveOpen   = g_bts724g_fault_status.bts724g_fault_open[VALVE_22IN]   ? 1u : 0u;
    PSWErr22InValveShort  = g_bts724g_fault_status.bts724g_fault_short[VALVE_22IN]  ? 1u : 0u;
    PSWErr22OutValveOpen  = g_bts724g_fault_status.bts724g_fault_open[VALVE_22OUT]  ? 1u : 0u;
    PSWErr22OutValveShort = g_bts724g_fault_status.bts724g_fault_short[VALVE_22OUT] ? 1u : 0u;

    /* TR_ASR: 新 RTE 已删除 ASR 阀故障字段, 仍按驱动状态更新, 仅内部/调试使用 */
    //PSWErrASRValveOpenX  = g_bts724g_fault_status.bts724g_fault_open[VALVE_TR_ASR]  ? 1u : 0u;
    //PSWErrASRValveShortX = g_bts724g_fault_status.bts724g_fault_short[VALVE_TR_ASR] ? 1u : 0u;

    /* ---- 6. 芯片级故障 (bts724g) ---- */
    PSWErrOpenDrive724_U6  = (g_bts724g_fault_status.bts724g_chip_fault_type[0] == VALVE_FAULT_OPEN)  ? 1u : 0u;
    PSWErrOpenDrive724_U9  = (g_bts724g_fault_status.bts724g_chip_fault_type[1] == VALVE_FAULT_OPEN)  ? 1u : 0u;
    PSWErrOpenDrive724_U12 = (g_bts724g_fault_status.bts724g_chip_fault_type[2] == VALVE_FAULT_OPEN)  ? 1u : 0u;
    PSWErrOpenDrive724_U13 = (g_bts724g_fault_status.bts724g_chip_fault_type[3] == VALVE_FAULT_OPEN)  ? 1u : 0u;
     PSWErrOpenDrive724_U19 = (g_bts724g_fault_status.bts724g_chip_fault_type[4] == VALVE_FAULT_OPEN)  ? 1u : 0u;

    PSWErrShortDrive724_U6  = (g_bts724g_fault_status.bts724g_chip_fault_type[0] == VALVE_FAULT_SHORT) ? 1u : 0u;
    PSWErrShortDrive724_U9  = (g_bts724g_fault_status.bts724g_chip_fault_type[1] == VALVE_FAULT_SHORT) ? 1u : 0u;
    PSWErrShortDrive724_U12 = (g_bts724g_fault_status.bts724g_chip_fault_type[2] == VALVE_FAULT_SHORT) ? 1u : 0u;
    PSWErrShortDrive724_U13 = (g_bts724g_fault_status.bts724g_chip_fault_type[3] == VALVE_FAULT_SHORT) ? 1u : 0u;
    PSWErrShortDrive724_U19 = (g_bts724g_fault_status.bts724g_chip_fault_type[4] == VALVE_FAULT_SHORT) ? 1u : 0u;
    /* ---- 7. 芯片总故障  ---- */
    {
        uint8_t chip_sum = PSWErrOpenDrive724_U6  + PSWErrOpenDrive724_U9+ PSWErrOpenDrive724_U12 + PSWErrOpenDrive724_U13 + PSWErrOpenDrive724_U19 + \
						PSWErrShortDrive724_U6 + PSWErrShortDrive724_U9 + PSWErrShortDrive724_U12 + PSWErrShortDrive724_U13 + PSWErrShortDrive724_U19;
                        PSWErrDriveChip = (chip_sum > 0u) ? 1u : 0u;
    }

    /* ---- 8. EEPROM (rte_psw.c EEPROM 函数写 PSWfErrEEprom; Relay 由 relay_diag 填入) ---- */
}

/* ========================================================================== */
/*  PSWDataToRte() — 从 PSW* 中间变量读取, 验证后写入 RTE                        */
/* ========================================================================== */
void PSWDataToRte(void)
{
    const bool vpwr_ok = (PSWvIgn >= 180u && PSWvIgn <= 320u);

    /* ---- 0. 任务执行时间 (由 rcrd_task_start/calc_task_time 填充 PSWTaskTime) ---- */
    RTETaskTime = PSWTaskTime;

    /* ---- 1. VPOWER 电压 + 报警 (P7.2 AN10, 不受 vpwr_ok 影响) ---- */
    RTEvIgn = PSWvIgn;

    RTEfPSWErr.RTEfErrVpwrHi = PSWErrVpwrHi;
    RTEfPSWErr.RTEfErrVpwrLo = PSWErrVpwrLo;
    RTEfPSWErr.RTEfErrVbatLo = 0u;
    RTEfPSWErr.RTEfErrVbatHi = 0u;
    RTEfPSWErr.RTEfErrRelay   = PSWErrRelay;

    /* ---- 2. 压力传感器 (VPOWER 异常 → 值置 0xFFFF, 故障保持旧值) ---- */
    if (vpwr_ok)
    {
        uint8_t err_front = PSWfErrPreX2_7;
        uint8_t err_rear  = PSWfErrPreX3_10;

        RTEfErrPreX4_14  = err_front;
        RTEfErrPreX4_15 = err_rear;

        RTEBrkPreX4_14  = (err_front != 0u) ? DATA_INVALID : PSWBrkPreX2_7;
        RTEBrkPreX4_15 = (err_rear  != 0u) ? DATA_INVALID : PSWBrkPreX3_10;

        /* 前/后桥压力故障的映射不在本层:
         * 新 RTE 已把 RTEfErrPreF/RTEfErrPreR 从 PSWErr_Struct 移到 COM 层全局 (RTE.h:245-246),
         * 映射依据 RTEfPressCfg (COM 段变量, 取值见表 RTE.h:258-263) 由 COM 完成。
         * 本层只上报口级: RTEBrkPreX4_14/15 + RTEfErrPreX4_14/15。 */
    }
    else
    {
        RTEBrkPreX4_14  = DATA_INVALID;
        RTEBrkPreX4_15 = DATA_INVALID;
    }

    /* ---- 3. 轮速传感器故障 ---- */
    {
        RTEfPSWErr.RTEfErrWssOpenFL  = PSWErrWssOpenFL;
        RTEfPSWErr.RTEfErrWssShortFL = PSWErrWssShortFL;
        RTEfPSWErr.RTEfErrWssGapFL   = PSWErrWssGapFL;

        RTEfPSWErr.RTEfErrWssOpenFR  = PSWErrWssOpenFR;
        RTEfPSWErr.RTEfErrWssShortFR = PSWErrWssShortFR;
        RTEfPSWErr.RTEfErrWssGapFR   = PSWErrWssGapFR;

        RTEfPSWErr.RTEfErrWssOpenRL  = PSWErrWssOpenRL;
        RTEfPSWErr.RTEfErrWssShortRL = PSWErrWssShortRL;
        RTEfPSWErr.RTEfErrWssGapRL   = PSWErrWssGapRL;

        RTEfPSWErr.RTEfErrWssOpenRR  = PSWErrWssOpenRR;
        RTEfPSWErr.RTEfErrWssShortRR = PSWErrWssShortRR;
        RTEfPSWErr.RTEfErrWssGapRR   = PSWErrWssGapRR;

        RTEfPSWErr.RTEfErrWssOpenXL  = PSWErrWssOpenXL;
        RTEfPSWErr.RTEfErrWssShortXL = PSWErrWssShortXL;
        RTEfPSWErr.RTEfErrWssGapXL   = PSWErrWssGapXL;

        RTEfPSWErr.RTEfErrWssOpenXR  = PSWErrWssOpenXR;
        RTEfPSWErr.RTEfErrWssShortXR = PSWErrWssShortXR;
        RTEfPSWErr.RTEfErrWssGapXR   = PSWErrWssGapXR;
    }

    /* ---- 4. 轮速值 (wheel_speed 已为 mm/s) + 故障掩蔽 ---- */
    {
        uint16_t ws[6];
        ws[0] = PSWWheelSpeedFL;
        ws[1] = PSWWheelSpeedFR;
        ws[2] = PSWWheelSpeedRL;
        ws[3] = PSWWheelSpeedRR;
        ws[4] = PSWWheelSpeedXL;
        ws[5] = PSWWheelSpeedXR;

        RTEWheelSpeedFL = ((uint8_t)(PSWErrWssOpenFL + PSWErrWssShortFL + PSWErrWssGapFL) != 0u) ? DATA_INVALID : ws[0];
        RTEWheelSpeedFR = ((uint8_t)(PSWErrWssOpenFR + PSWErrWssShortFR + PSWErrWssGapFR) != 0u) ? DATA_INVALID : ws[1];
        RTEWheelSpeedRL = ((uint8_t)(PSWErrWssOpenRL + PSWErrWssShortRL + PSWErrWssGapRL) != 0u) ? DATA_INVALID : ws[2];
        RTEWheelSpeedRR = ((uint8_t)(PSWErrWssOpenRR + PSWErrWssShortRR + PSWErrWssGapRR) != 0u) ? DATA_INVALID : ws[3];
        RTEWheelSpeedXL = ((uint8_t)(PSWErrWssOpenXL + PSWErrWssShortXL + PSWErrWssGapXL) != 0u) ? DATA_INVALID : ws[4];
        RTEWheelSpeedXR = ((uint8_t)(PSWErrWssOpenXR + PSWErrWssShortXR + PSWErrWssGapXR) != 0u) ? DATA_INVALID : ws[5];
    }

    /* ---- 5. 阀故障 + 6. 芯片级故障 (VPOWER 异常 → 不写入, 旧值保持) ---- */
    if (vpwr_ok)
    {
        RTEfPSWErr.RTEfErrInValOpenFL   = PSWErrInValOpenFL;
        RTEfPSWErr.RTEfErrInValShortFL  = PSWErrInValShortFL;
        RTEfPSWErr.RTEfErrOutValOpenFL  = PSWErrOutValOpenFL;
        RTEfPSWErr.RTEfErrOutValShortFL = PSWErrOutValShortFL;

        RTEfPSWErr.RTEfErrInValOpenFR   = PSWErrInValOpenFR;
        RTEfPSWErr.RTEfErrInValShortFR  = PSWErrInValShortFR;
        RTEfPSWErr.RTEfErrOutValOpenFR  = PSWErrOutValOpenFR;
        RTEfPSWErr.RTEfErrOutValShortFR = PSWErrOutValShortFR;

        RTEfPSWErr.RTEfErrInValOpenRL   = PSWErrInValOpenRL;
        RTEfPSWErr.RTEfErrInValShortRL  = PSWErrInValShortRL;
        RTEfPSWErr.RTEfErrOutValOpenRL  = PSWErrOutValOpenRL;
        RTEfPSWErr.RTEfErrOutValShortRL = PSWErrOutValShortRL;

        RTEfPSWErr.RTEfErrInValOpenRR   = PSWErrInValOpenRR;
        RTEfPSWErr.RTEfErrInValShortRR  = PSWErrInValShortRR;
        RTEfPSWErr.RTEfErrOutValOpenRR  = PSWErrOutValOpenRR;
        RTEfPSWErr.RTEfErrOutValShortRR = PSWErrOutValShortRR;

        RTEfPSWErr.RTEfErrInValOpenXL   = PSWErrInValOpenXL;
        RTEfPSWErr.RTEfErrInValShortXL  = PSWErrInValShortXL;
        RTEfPSWErr.RTEfErrOutValOpenXL  = PSWErrOutValOpenXL;
        RTEfPSWErr.RTEfErrOutValShortXL = PSWErrOutValShortXL;

        RTEfPSWErr.RTEfErrInValOpenXR   = PSWErrInValOpenXR;
        RTEfPSWErr.RTEfErrInValShortXR  = PSWErrInValShortXR;
        RTEfPSWErr.RTEfErrOutValOpenXR  = PSWErrOutValOpenXR;
        RTEfPSWErr.RTEfErrOutValShortXR = PSWErrOutValShortXR;

        /* 21口 = 前桥第2组阀, 按接插件引脚命名 (RTE 2026-09-10 改名:
         *   RTEfErr21ValveOpen/Short    -> RTEfErrX3_7InValveOpen/Short    (X3_7  进气)
         *   RTEfErr21OutValveOpen/Short -> RTEfErrX3_10OutValveOpen/Short   (X3_10 排气) */
        RTEfPSWErr.RTEfErrX3_7InValveOpen    = PSWErr21InValveOpen;
        RTEfPSWErr.RTEfErrX3_7InValveShort   = PSWErr21InValveShort;
        RTEfPSWErr.RTEfErrX3_10OutValveOpen  = PSWErr21OutValveOpen;
        RTEfPSWErr.RTEfErrX3_10OutValveShort = PSWErr21OutValveShort;

        /* 22口 = 后桥第2组阀, 按接插件引脚命名:
         *   RTEfErr22InValveOpen/Short  -> RTEfErrX4_16InValveOpen/Short   (X4_16 进气)
         *   RTEfErr22OutValveOpen/Short -> RTEfErrX2_16OutValveOpen/Short  (X2_16 排气) */
        RTEfPSWErr.RTEfErrX4_16InValveOpen   = PSWErr22InValveOpen;
        RTEfPSWErr.RTEfErrX4_16InValveShort  = PSWErr22InValveShort;
        RTEfPSWErr.RTEfErrX2_16OutValveOpen  = PSWErr22OutValveOpen;
        RTEfPSWErr.RTEfErrX2_16OutValveShort = PSWErr22OutValveShort;

        /* TR_ASR: 新 RTE 已删除 ASR 阀故障字段, 不再上报 (PSW 内部变量仍保留) */

        RTEfPSWErr.RTEfErrDriveChip = PSWErrDriveChip;
    }

    /* ---- 7. EEPROM/Relay (锁存故障, 不清零) ---- */
    {
        RTEfPSWErr.RTEfErrEEprom       = PSWfErrEEprom;
        RTEfPSWErr.RTEfErrEEpromOutBnd = PSWfErrEEpromOutBnd;
        RTEfPSWErr.RTEfErrRelay        = PSWErrRelay;
    }
}

/* 压力 10ms CAN 上报 (0x730) 已移至 test/test_deal.c (开关 test_cfg.h 的 TEST_PRESSURE_CAN) */
