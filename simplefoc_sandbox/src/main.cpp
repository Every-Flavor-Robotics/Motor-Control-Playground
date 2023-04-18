#include <SimpleFOC.h>

// Encoder libs
#include <sensors/MagneticSensorI2C.h>
#include "mt6701_sensor.h"
#include "encoders/calibrated/CalibratedSensor.h"
#include <Wire.h>

// Encoder pins
const int k_enc_sda = 21;
const int k_enc_scl = 22;
const int k_enc_cs  = 32;

// Motor pins
const int k_gpio_uh = 33;
const int k_gpio_ul = 25;
const int k_gpio_vh = 26;
const int k_gpio_vl = 27;
const int k_gpio_wh = 14;
const int k_gpio_wl = 12;

// target velocities will be calculated as a percentage of the max velocity
constexpr float max_velocity_rad_per_sec = 25.0f;

// Encoder, motor, and driver instances
MT6701Sensor encoder = MT6701Sensor();
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(k_gpio_uh, k_gpio_ul, k_gpio_vh, k_gpio_vl, k_gpio_wh, k_gpio_wl);
// Create a calibrated sensor instance and pass it the encoder
CalibratedSensor sensor_calibrated = CalibratedSensor(encoder);

// commander instance
Commander command = Commander(Serial);
void doTarget(char* cmd){command.motor(&motor, cmd);}

void setup()
{
    // start serial
    Serial.begin(115200);

    // Initialize encoder
    encoder.init(k_enc_scl, k_enc_sda, k_enc_cs);
    // link motor to sensor
    motor.linkSensor(&encoder);

    // Initialize driver
    driver.voltage_power_supply = 5.0;
    driver.voltage_limit = 5.0;
    driver.init();
    // Link driver to motor
    motor.linkDriver(&driver);

    // set motion control type to velocity
    motor.controller = MotionControlType::velocity;
    // Set Space Vector PWM modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    // set torque control type to voltage (default)
    motor.torque_controller = TorqueControlType::voltage;

    // set motor velocity, voltage, and current limits.

    motor.velocity_limit = 60.0;
    motor.voltage_limit = 7.0;
    motor.current_limit = 1.2;
    motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;


    // use monitoring
    motor.useMonitoring(Serial);


    // initialize motor
    motor.init();


    // note: (alex) setting calibration voltage should act as a fraction of the voltage limit or supply and reduce current draw,
    //       ie 3v calibration with 11v rail -> 3/11 volt used for calibration = pwm of 27% while running open loop
    //       *** it could use voltage limit instead, AND I'm not even sure this is working at all...
    sensor_calibrated.voltage_calibration = 1;
    sensor_calibrated.calibrate(motor);

    motor.linkSensor(&sensor_calibrated);


    // Init FOC
    motor.initFOC();

    // add command to commander
    command.add('M', doTarget, "target");

    _delay(1000);
}

void loop()
{
    motor.loopFOC();

    // this function can be run at much lower frequency than loopFOC()
    motor.move();

    // user communication
    command.run();

    // btw, running this is significantly slowing the execution down (will vibrate more)
    motor.monitor();


}
