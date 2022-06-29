#include "Arduino.h"
#include "SimpleFOC.h"
#include "BluetoothSerial.h"
#include "imu_helpers.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial bluetooth;

SPIClass SPI_0 = SPIClass(VSPI);
SPIClass SPI_1 = SPIClass(VSPI);
MagneticSensorSPI sensor0 = MagneticSensorSPI(AS5147_SPI, 17);
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, 16);

// Motor instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

// control algorithm parameters
// stabilisation pid
PIDController pid_stb = PIDController(30, 100, 1, 100, 3);
// velocity pid
PIDController pid_vel = PIDController(0.01, 0.03, 0, 100000, _PI / 10);
// velocity control filtering
LowPassFilter lpf_pitch_cmd = LowPassFilter(0.07);
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle = LowPassFilter(0.5);
LowPassFilter lpf_steering = LowPassFilter(0.1);

float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1; // 1 V
int state = 1; // 1 on / 0 off

// motion control tunning using commander
//Commander commander = Commander(Serial);
Commander commander = Commander(bluetooth);
void cntStab(char* cmd) {  commander.pid(&pid_stb, cmd);}
void cntMove(char* cmd) {  commander.pid(&pid_vel, cmd);}
void lpfPitch(char* cmd) {  commander.lpf(&lpf_pitch_cmd, cmd);}
void lpfSteering(char* cmd) {  commander.lpf(&lpf_steering, cmd);}
void lpfThrottle(char* cmd) {  commander.lpf(&lpf_throttle, cmd);}

void setup() {
  Serial.begin(115200);
  bluetooth.begin("foc_balancer");

  if ( !initIMU() ) {
    Serial.println(F("IMU connection problem... Disabling!"));
    bluetooth.println(F("IMU connection problem... Disabling!"));
    return;
  }
  _delay(1000);

  SPI_0.begin(19, 18, 15, 17);
  SPI_1.begin(19, 18, 15, 16);

  sensor0.init(&SPI_0);
  sensor1.init(&SPI_1);
  // link the motor to the sensor
  motor1.linkSensor(&sensor0);
  motor2.linkSensor(&sensor1);

  // power supply voltage [V]
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  // set control loop type to be used
  // using voltage torque mode
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // enable monitoring
//  motor1.useMonitoring(Serial);
  motor1.useMonitoring(bluetooth);
//  motor2.useMonitoring(Serial);
  motor2.useMonitoring(bluetooth);

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC
  motor1.initFOC(3.01, Direction::CCW);
  motor2.initFOC(3.63, Direction::CCW);

  // add the configuration commands
  commander.add('A', cntStab, "pid stab");
  commander.add('B', cntMove, "pid vel");
  commander.add('C', lpfThrottle, "lpf vel command");
  commander.add('D', lpfPitch, "lpf throttle");
  commander.add('E', lpfSteering, "lpf steering");

  Serial.println(F("Balancing robot ready!"));
  bluetooth.println(F("Balancing robot ready!"));
}

void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  if (!state) {
    motor1.target = 0;
    motor2.target = 0;
  } else if ( hasDataIMU() ) {
    float pitch = getPitchIMU();
//    Serial.printf("pitch: %f\n", pitch * 180 / M_PI);
    float target_pitch = lpf_pitch_cmd(pid_vel((motor1.shaft_velocity + motor2.shaft_velocity) / 2 - lpf_throttle(throttle)));
    // calculate the target voltage
    float voltage_control = pid_stb(target_pitch - pitch);

    // filter steering
    float steering_adj = lpf_steering(steering);
    // set the tergat voltage value
    motor1.target = voltage_control + steering_adj;
    motor2.target = -voltage_control + steering_adj;
  }

  commander.run();
}