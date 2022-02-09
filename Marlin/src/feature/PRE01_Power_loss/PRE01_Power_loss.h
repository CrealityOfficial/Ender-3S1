// #ifndef __PRE01_POWER_LOSS_H
// #define __PRE01_POWER_LOSS_H
#pragma once

#include "../../inc/MarlinConfig.h"
#include "../../MarlinCore.h"
#include "../../module/temperature.h"

#ifdef CREALITY_ENDER3_2021

typedef enum {
    HB220V = 1,
    HB110V
}Voltage_e;

typedef temp_info_t PowerInfo_t;

class PRE01PowerLoss
{
private:
    PRE01PowerLoss() {memset(&Info, 0, sizeof(PowerInfo_t));}
    PowerInfo_t Info;
    static PRE01PowerLoss *Instance;

public:
    ~PRE01PowerLoss() { if (Instance) delete Instance;}
    void Init() {
        HAL_ANALOG_SELECT(POWER_DETECTION_PIN);
        //OUT_WRITE(BACKPOWER_CTRL_PIN, HIGH);    /*高电平打开电容充电*/
        OUT_WRITE(BACKPOWER_CTRL_PIN, LOW);    /*高电平打开电容充电*/
    }

    static PRE01PowerLoss *GetInstance() {return Instance;}
    void DoSomeThing();
    PowerInfo_t *UpDateInfo() { return &Info;}
};
//GetInstance()->UpDateInfo()

extern void my_report_logical_position(const xyze_pos_t &rpos);

#endif /*#ifdef CREALITY_ENDER3_2021*/

//#endif /*#ifndef __PRE01_POWER_LOSS_H*/