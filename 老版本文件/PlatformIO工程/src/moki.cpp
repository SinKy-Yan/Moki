#include "Moki.h"
#include "Arduino.h"
#include <SimpleFOC.h>

#ifndef m1_offset
#warning 未设置电机1的机械偏差,将会在开机的时候自动检测
#define m1_offset -1
#endif

#ifndef m2_offset
#warning 未设置电机2的机械偏差,将会在开机的时候自动检测
#define m2_offset -1
#endif

/* 定义IO口,不要动 */
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor1 = BLDCMotor(11, 12.5, 74);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(13, 12, 27, 14);
BLDCMotor motor2 = BLDCMotor(11, 12.5, 74);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(15, 0, 4, 2);

/* 串口控制 */
Commander command = Commander(Serial);
void onMotor1(char *cmd) { command.motor(&motor1, cmd); }
void onMotor2(char *cmd) { command.motor(&motor2, cmd); }

Moki::Moki(int number_of_motor)
{
    this->number_of_motor = number_of_motor;
}

/* 初始化Moki(无传入变量) */
void Moki::setup()
{
    /* ESP32通信接口设置 */
    setCpuFrequencyMhz(Cpu_Frequency);
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 21, 22);

    /* 板载LED设置 */
    // xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, NULL, 1);
    // ledcSetup(1, 1000, 8);  // 设置LEDC通道8频率为1,分辨率为10位,即占空比可选0~1023
    // ledcAttachPin(2, 1); // 设置LEDC通道8在IO14上输出
    // ledcWrite(1, 1000);

    command.add('A', onMotor1, "设定电机1");
    command.add('B', onMotor2, "设定电机2");

    if (this->number_of_motor == 1)
    {
        init_m1();
    }
    else if (this->number_of_motor == 2)
    {
        init_m1();
        init_m2();
    }
}

/* 启动FOC控制(无传入变量) */
void Moki::run(int number_of_example)
{
    if (this->number_of_motor == 1)
    {
        run_m1();
    }
    else if (this->number_of_motor == 2)
    {
        run_m1();
        run_m2();
    }
    command.run();
}

void Moki::run_m1()
{
    // if (sensor1.getAngle() > 0.00 or sensor1.getAngle() < 2.00)
    // {
    //     motor1.voltage_limit = 0;
    //     Serial.println(sensor1.getAngle() - 0);
    // }
    // else if (sensor1.getAngle() < 0.00)
    // {
    //     motor1.voltage_limit = 2;
    //     motor1.move(0);
    // }
    // else if (sensor1.getAngle() > 2.00)
    // {
    //     motor1.voltage_limit = 2;
    //     motor1.move(2);
    // }

    motor1.loopFOC();
    motor1.move();
    // Serial.println(sensor1.getAngle());
}

void Moki::run_m2()
{
    motor2.loopFOC();
    motor2.move();
}

void Moki::init_m1()
{
    Wire.begin(25, 26, (uint32_t)i2c_Frequency);
    sensor1.init();
    motor1.linkSensor(&sensor1);
    driver1.voltage_power_supply = power_supply_voltage;
    driver1.init();
    motor1.linkDriver(&driver1);
    motor1.foc_modulation = FOCModulationType::SinePWM;
    motor1.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::Motion_Control_Type;
    // Serial.println("已设置m1的驱动器和传感器");

    motor1.PID_velocity.P = 0.1;
    motor1.PID_velocity.I = 0.2;
    motor1.PID_velocity.D = 0;
    motor1.PID_velocity.output_ramp = 50;
    motor1.LPF_velocity.Tf = 0.1;
    motor1.velocity_limit = 20;
    motor1.voltage_limit = motor_voltage_limit;
    motor1.current_limit = 0.3;
    motor1.P_angle.P = 20;
    motor1.P_angle.I = 0;
    motor1.P_angle.D = 0;
    motor1.P_angle.output_ramp = 500;
    motor1.LPF_angle.Tf = 0;
    // Serial.println("已设置m1的控制参数");

    motor1.init();

    if (m1_offset == -1)
    {
        Serial.println("未设置m1机械偏差,开始自检");
        motor1.initFOC();
    }

    else
    {
        Serial.println("已设置m1机械偏差,跳过自检");
        motor1.initFOC(m1_offset, Direction::CCW);
    }
    Serial.println("m1初始化完成");

    motor1.target = Target;
}

void Moki::init_m2()
{
    Wire1.begin(33, 32, (uint32_t)i2c_Frequency);
    sensor2.init(&Wire1);
    motor2.linkSensor(&sensor2);
    driver2.voltage_power_supply = power_supply_voltage;
    driver2.init();
    motor2.linkDriver(&driver2);
    motor2.foc_modulation = FOCModulationType::SinePWM;
    motor2.torque_controller = TorqueControlType::voltage;
    motor2.controller = MotionControlType::Motion_Control_Type;
    // Serial.println("已设置m2的驱动器和传感器");

    motor2.PID_velocity.P = 0.1;
    motor2.PID_velocity.I = 0.2;
    motor2.PID_velocity.D = 0;
    motor2.PID_velocity.output_ramp = 50;
    motor2.LPF_velocity.Tf = 0.1;
    motor2.velocity_limit = 20;
    motor2.voltage_limit = 6;
    motor2.current_limit = 0.3;
    motor2.P_angle.P = 20;
    motor2.P_angle.I = 0;
    motor2.P_angle.D = 0;
    motor2.P_angle.output_ramp = 500;
    motor2.LPF_angle.Tf = 0;
    // Serial.println("已设置m2的控制参数");

    motor2.init();

    if (m1_offset == -1)
    {
        Serial.println("未设置m2机械偏差,开始自检");
        motor2.initFOC();
    }

    else
    {
        Serial.println("已设置m2机械偏差,跳过自检");
        motor2.initFOC(m2_offset, Direction::CCW);
    }

    Serial.println("m2初始化完成");

    motor2.target = Target;
}