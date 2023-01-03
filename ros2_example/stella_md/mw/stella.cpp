#include "MW_value.hpp"
#include "MW_serial.hpp"

struct Motor_Configuration_value stella_n1;
struct Differential_Mobile_Robot_value Differential_MobileRobot_SetValue;

int stella_setting(int select);
int Differential_MobileRobot_Setting();
int stella_setting_check(int channel);
int Differential_MobileRobot_setting_check();

int stella_setting_check(int channel)
{
    if (stella_n1.encoder_ppr[channel] != MyMotorConfiguration.encoder_ppr[channel])
        return 0;
    if (stella_n1.Max_voltage[channel] != MyMotorConfiguration.Max_voltage[channel])
        return 0;
    if (stella_n1.Max_current[channel] != MyMotorConfiguration.Max_current[channel])
        return 0;

    if (stella_n1.Max_velocity[channel] != MyMotorConfiguration.Max_velocity[channel])
        return 0;
    if (stella_n1.Max_velocity[channel] != MyMotorConfiguration.Max_velocity[channel])
        return 0;
    if (stella_n1.Acceleration[channel] != MyMotorConfiguration.Acceleration[channel])
        return 0;
    if (stella_n1.Deceleration[channel] != MyMotorConfiguration.Deceleration[channel])
        return 0;

    if (stella_n1.overvoltage_limit[channel] != MyMotorConfiguration.overvoltage_limit[channel])
        return 0;
    if (stella_n1.undervoltage_limit[channel] != MyMotorConfiguration.undervoltage_limit[channel])
        return 0;
    if (stella_n1.overcurrent_limit[channel] != MyMotorConfiguration.overcurrent_limit[channel])
        return 0;

    if (stella_n1.direction[channel] != MyMotorConfiguration.direction[channel])
        return 0;

    if (stella_n1.velocity_p_gain[channel] != MyMotorConfiguration.velocity_p_gain[channel])
        return 0;
    if (stella_n1.velocity_i_gain[channel] != MyMotorConfiguration.velocity_i_gain[channel])
        return 0;

    return 1;
}

int Differential_MobileRobot_setting_check()
{
    if (Differential_MobileRobot_SetValue.axle_length != Differential_MobileRobot.axle_length)
        return 0;
    if (Differential_MobileRobot_SetValue.wheel_radius != Differential_MobileRobot.wheel_radius)
        return 0;
    if (Differential_MobileRobot_SetValue.gear_ratio != Differential_MobileRobot.gear_ratio)
        return 0;

    return 1;
}
int stella_setting(int select)
{
    switch (select)
    {
    case 0:
        // 모터사양 참고
        stella_n1.encoder_ppr[channel_1] = 54000;
        stella_n1.encoder_ppr[channel_2] = 54000;

        stella_n1.Max_voltage[channel_1] = 12.0;
        stella_n1.Max_voltage[channel_2] = 12.0;

        stella_n1.Max_current[channel_1] = 2.0;
        stella_n1.Max_current[channel_2] = 2.0;

        stella_n1.Max_velocity[channel_1] = 300.0;
        stella_n1.Max_velocity[channel_2] = 300.0;

        stella_n1.Acceleration[channel_1] = 280.0;
        stella_n1.Acceleration[channel_2] = 280.0;

        stella_n1.Deceleration[channel_1] = 280.0;
        stella_n1.Deceleration[channel_2] = 280.0;

        // 제어기 안전
        stella_n1.overvoltage_limit[channel_1] = 15.0;
        stella_n1.overvoltage_limit[channel_2] = 15.0;

        stella_n1.undervoltage_limit[channel_1] = 9.0;
        stella_n1.undervoltage_limit[channel_2] = 9.0;

        stella_n1.overcurrent_limit[channel_1] = 3.0;
        stella_n1.overcurrent_limit[channel_2] = 3.0;

        // 하드웨어 참고
        stella_n1.direction[channel_1] = Not_Change;
        stella_n1.direction[channel_2] = Reverse_Direction;

        stella_n1.velocity_p_gain[channel_1] = 1.6;
        stella_n1.velocity_p_gain[channel_2] = 1.6;

        stella_n1.velocity_i_gain[channel_1] = 0.001;
        stella_n1.velocity_i_gain[channel_2] = 0.001;

        // 로봇 기구학 셋팅
        Differential_MobileRobot_SetValue.axle_length = 0.0012;  //(m)
        Differential_MobileRobot_SetValue.wheel_radius = 0.0012; //(m)
        Differential_MobileRobot_SetValue.gear_ratio = 0.1234;   // 모터회전수 / 바퀴회전수 (감속비율)

        // 적용
        Set_MotorConfiguration(channel_1, &stella_n1);
        Set_MotorConfiguration(channel_2, &stella_n1);

        Set_Differential_MobileRobot_value(&Differential_MobileRobot_SetValue);
    }

    return 1;
}

int Robot_Setting(int choice)
{
    stella_setting(choice);

    int res = stella_setting_check(channel_1) && stella_setting_check(channel_2) && Differential_MobileRobot_setting_check();

    if (!res)
    {
        printf("\n\n Robot Setting ... \n\n ");

        FunctionMotorControlSystem(SAVE_CONFIG_FLASH);

        Get_Differential_MobileRobot_value();

        Get_MotorConfiguration(channel_1);
        Get_MotorConfiguration(channel_2);

        InformationsPrintf();

        return 1;
    }

    return 1;
}