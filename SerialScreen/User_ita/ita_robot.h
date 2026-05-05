#ifndef ITA_ROBOT_H
#define ITA_ROBOT_H

#include "crt_chassis.h"
#include "dvc_nvidiaorin.h"
#include "dvc_sbus.h"
#include "dvc_serialscreen.h"

/**
 * @brief SBUS控制数据来源
 *
 */
enum Enum_SBUS_Control_Type
{
    SBUS_Control_Type_NONE = 0,
    SBUS_Control_Type_REMOTE ,
    
};

enum Enum_Control_Source
{
    SBUS_Control,
    Control_DISABLE,
};

// 添加活动控制器枚举类型
enum Enum_Active_Controller
{
    Controller_NONE = 0,
    Controller_SBUS,
    Controller_Orin,
};

class Class_Chariot
{
public:
    Class_SBUS SBUS;
    Class_Orin Orin;
    Class_Omni_Chassis Chassis;
    Class_SerialScreen SerialScreen;
    
    void Init(float __Dead_Zone = 0);
    void TIM_Control_Callback();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_101ms_Alive_PeriodElapsedCallback();

    void Judge_SBUS_Control_Type();
    void Judge_Active_Controller();
    void Judge_SerialScreen_Status();

    void Control_Chassis();
    void Control_SerialScreen();

protected:
    // SBUS控制数据来源
    Enum_SBUS_Control_Type SBUS_Control_Type = SBUS_Control_Type_NONE;

    // 当前活动的控制器
    Enum_Active_Controller Active_Controller = Controller_NONE;

    Enum_Control_Source Control_Source = Control_DISABLE;

    // 遥控器拨动的死区, 0~1
    float Dead_Zone;
};
#endif