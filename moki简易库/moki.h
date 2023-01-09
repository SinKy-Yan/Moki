#include <Arduino.h>
#ifndef Moki_h
#define Moki_h
#define VERSION = 0.1

/* 自定义用的参数 */
// #define m1_offset 4.71               /* 电机1的机械偏差,不知道的话设为-1 */
// #define m2_offset 5.47               /* 电机2的机械偏差,不知道的话设为-1 */
#define power_supply_voltage 10      /* 供电电压 */
#define Cpu_Frequency 80             /* ESP32的CPU频率 */
#define i2c_Frequency 400000         /* i2c的速率 */
#define Motion_Control_Type velocity /* 默认控制模式(torque、velocity、angle) */
#define Target 5                     /* 初始化之后的电机目标 */
#define motor_voltage_limit 6        /* 电机电压限制 */

/* Moki驱动库 */
class Moki 
{
    private:
        void run_m1();
        void run_m2();
        void init_m1();
        void init_m2();
        int number_of_motor;

    public:
        void setup();
        void run(int number_of_example);
        Moki(int number_of_motor);
};
#endif
