#include "ita_robot.h"
/**
 * @file ita_robot.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 人机交互控制逻辑
 * @version 1.1
 * @date 2023-08-29
 * @date 2024-01-17
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static bool SBUS_Has_Remote_Input(Class_SBUS &sbus)
{
    return (sbus.Get_Left_X() != 0.0f ||
            sbus.Get_Left_Y() != 0.0f ||
            sbus.Get_Right_X() != 0.0f ||
            sbus.Get_Right_Y() != 0.0f );
}

/**
 * @brief 底盘，云台，发射机构初始化
 *
 */
void Class_Chariot::Init(float __Dead_Zone)
{
    SBUS.Init(&huart5);
    Orin.Init(&hfdcan2);
    SerialScreen.Init(&huart1);

    Chassis.Init();

    Dead_Zone = __Dead_Zone;
}

/**
 * @brief 101ms定时任务
 * @brief 上位机Orin存活检测用
 */
void Class_Chariot::TIM_101ms_Alive_PeriodElapsedCallback()
{
    Orin.TIM_101ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 100ms定时任务
 *
 */
void Class_Chariot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    SBUS.TIM1msMod50_Alive_PeriodElapsedCallback();
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    SerialScreen.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    if (SBUS.Get_SBUS_Status() == SBUS_Status_DISABLE)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
        return;
    }
    if (SBUS.Get_Switch_C() == SBUS_Switch_Status_DOWN &&
        Orin.Get_Status() == Orin_Status_DISABLE)    
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
        return;
    }    
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_ENABLE);
}
/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_SBUS_Control_Type()
{
    if (SBUS_Has_Remote_Input(SBUS) == true)
    {
        SBUS_Control_Type = SBUS_Control_Type_REMOTE;
    }
    else
    {
        SBUS_Control_Type = SBUS_Control_Type_NONE;
    }
}

/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_Active_Controller()
{
    // 检查SBUS是否有输入
    Judge_SBUS_Control_Type();

    // 判断当前活动的控制器
    if (SBUS.Get_Switch_B() == SBUS_Switch_Status_DOWN && 
        SBUS.Get_SBUS_Status() == SBUS_Status_ENABLE)
    {
        Active_Controller = Controller_SBUS;
    }
    else if (SBUS.Get_Switch_B() == SBUS_Switch_Status_UP &&
             Orin.Get_Status() == Orin_Status_ENABLE)
    {
        Active_Controller = Controller_Orin;
    }    
    else
    {
        Active_Controller = Controller_NONE;
    }
}

/**
 * @brief 获取串口屏状态
 *
 */void Class_Chariot::Judge_SerialScreen_Status()
{
    if (SBUS.Get_Switch_D() == SBUS_Switch_Status_DOWN)
    {
        SerialScreen.Set_Screen_Status(SCREEN_STATUS_DISABLE);
    }
    else if (SBUS.Get_Switch_D() == SBUS_Switch_Status_UP)
    {
        SerialScreen.Set_Screen_Status(SCREEN_STATUS_ENABLE);
    }
}

/**
 * @brief 底盘，云台，发射机构控制逻辑
 *
 */
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    Judge_Active_Controller();

    /************************************上位机控制逻辑*********************************************/
    if (Active_Controller == Controller_Orin)
    {
        float orin_vx = Orin.Get_Target_Velocity_X();
        float orin_vy = Orin.Get_Target_Velocity_Y();
        float orin_omega = Orin.Get_Target_Omega();

        if (SBUS.Get_Switch_A() == SBUS_Switch_Status_DOWN) // 底盘失能
        {
			Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        else if (SBUS.Get_Switch_A() == SBUS_Switch_Status_UP) // 底盘随动，遥控器专用
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chassis.Set_Target_Velocity_X(orin_vx);
            Chassis.Set_Target_Velocity_Y(orin_vy);
            Chassis.Set_Target_Omega(orin_omega);
        }

    }
    /************************************遥控器控制逻辑*********************************************/

    else if (Active_Controller == Controller_SBUS && SBUS_Control_Type == SBUS_Control_Type_REMOTE)
    {
        float sbus_l_x, sbus_l_y, sbus_yaw;
        float chassis_velocity_x = 0, chassis_velocity_y = 0;
        static float chassis_omega = 0;
        static float chassis_angle = 0;
        // 排除遥控器死区
        sbus_l_x = (Math_Abs(SBUS.Get_Left_Y()) > Dead_Zone) ? SBUS.Get_Left_Y() : 0;
        sbus_l_y = (Math_Abs(SBUS.Get_Left_X()) > Dead_Zone) ? SBUS.Get_Left_X()*(-1.0f) : 0;
        // yaw和xy的死区是否相同存疑
        sbus_yaw = (Math_Abs(SBUS.Get_Right_X()) > Dead_Zone) ? SBUS.Get_Right_X()*(-1.0f) : 0;
        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = sbus_l_x * sqrt(1.0f - sbus_l_y * sbus_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = sbus_l_y * sqrt(1.0f - sbus_l_x * sbus_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
        chassis_omega = sbus_yaw * Chassis.Get_Omega_Max();
        chassis_angle += chassis_omega;
        // 键盘遥控器操作逻辑
        if (SBUS.Get_Switch_A() == SBUS_Switch_Status_DOWN) // 底盘失能
        {
			Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        else if (SBUS.Get_Switch_A() == SBUS_Switch_Status_UP) // 底盘随动，遥控器专用
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
            Chassis.Set_Target_Omega(chassis_omega);
        }
    }
}

/**
 * @brief 控制串口屏
 *
 */
uint16_t Current_time = 0;
uint16_t Last_time = 0;

void Class_Chariot::Control_SerialScreen()
{
    Judge_SerialScreen_Status();

    Current_time = DWT_GetCurrentTimeS();
    
    if (Current_time - Last_time >= 1 && SerialScreen.Get_Screen_Status() == SCREEN_STATUS_ENABLE)
    {
        Last_time = Current_time;
        if (SBUS.Get_Switch_C() == SBUS_Switch_Status_UP) 
        {
            SerialScreen.Jump_To_Page(SCREEN_PAGE_2);
        }
        else if (SBUS.Get_Switch_C() == SBUS_Switch_Status_DOWN) 
        {
            SerialScreen.Jump_To_Page(SCREEN_PAGE_0);
        }
        else if (SBUS.Get_Switch_C() == SBUS_Switch_Status_MIDDLE) 
        {
            SerialScreen.Jump_To_Page(SCREEN_PAGE_1);
        }
    }
    else return;


}

void Class_Chariot::TIM_Control_Callback()
{
    Judge_Active_Controller();
    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();

    Control_SerialScreen();
    
}
