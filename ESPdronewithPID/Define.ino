
//Configuration File

#define BUILT_IN_LED 16

//----------PID----------//

#include <PID_v1.h>


#define ROLL_PID_MIN  -400.0
#define ROLL_PID_MAX  400.0

#define PITCH_PID_MIN  -400.0
#define PITCH_PID_MAX  400.0

#define YAW_PID_MIN  -300.0
#define YAW_PID_MAX  300.0


float prRateKp=0.5,   prRateKi=0.0,    prRateKd=0.0;
float yawRateKp=0.25,  yawRateKi=0.0,   yawRateKd=0.0;
float prStabKp=4.5,   prStabKi=0.0,    prStabKd=0.0;
float yawStabKp=0.0,  yawStabKi=0.0,   yawStabKd=0.0;



// PID variables
double pidRateIn[3], pidRateOut[3], pidRateSetpoint[3];
double pidStabIn[3], pidStabOut[3], pidStabSetpoint[3];
//The Rate controller was reversed. Noted.

PID roll_rate_controller(&pidRateIn[2],  &pidRateOut[2], &pidRateSetpoint[2], prRateKp,  prRateKi,  prRateKd,  DIRECT);
PID pitch_rate_controller(&pidRateIn[1], &pidRateOut[1], &pidRateSetpoint[1], prRateKp,  prRateKi,  prRateKd,  DIRECT);
PID yaw_rate_controller(&pidRateIn[0],   &pidRateOut[0], &pidRateSetpoint[0], yawRateKp, yawRateKi, yawRateKd, DIRECT);

PID roll_stab_controller(&pidStabIn[2],  &pidStabOut[2], &pidStabSetpoint[2], prStabKp,  prStabKi,  prStabKd, DIRECT);
PID pitch_stab_controller(&pidStabIn[1], &pidStabOut[1], &pidStabSetpoint[1], prStabKp,  prStabKi,  prStabKd, DIRECT);
PID yaw_stab_controller(&pidStabIn[0],   &pidStabOut[0], &pidStabSetpoint[0], yawStabKp,  yawStabKi,  yawStabKd, DIRECT);




//----------IMU----------//

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define IMU_GROUND 10

MPU6050 imu;

// IMU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t imuIntStatus;   // holds actual interrupt status byte from IMU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float offset[3];
float yaw_set = 0.0;
//Gyro values array
VectorInt16 g;
float gyroAverage = 0.0;

volatile bool imuInterrupt = false;     // indicates whether imu interrupt pin has gone high

//IMU stabilization variables
float yawPrevious = 0;
bool yawCompare = false;
int yawCompareCounter = 0;
byte yawComparePrint = 1;
byte imuStable = 0;

//Offsets
byte flagSetOffsets = 1;
float pitchOffset = 0.6;
float rollOffset = -1.2;
float yawOffset = 0;

//----------MOTORS----------//

//Motors arming throttle limit
#define ARMING_THR_LIMIT 1080

//Motor Output Parameters
const int motor[4] = {13, 0, 12, 14}; // LED's corresponding to future Motor output pins
double motorValue[4] = {0, 0, 0, 0};

//Servo library params for PWM 1000-2000
#include <Servo.h>
#define PWM_MIN 1000
#define PWM_MAX 2000
Servo esc[4];


//----------WIFI----------//

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <string.h>

#define NO_OF_PORTS 2

//Local AP SSID and Password
const char WiFiApPwd[20] = "12345677";
const char WiFiApSsid[20] = "ESPDroneNew";

//UDP Parameters
unsigned int localPort[NO_OF_PORTS] = {7000, 5000};
byte packetBuffer[512];
WiFiUDP UdpPort[NO_OF_PORTS];

 
//Input Strings
char pidType[10];
char thrStr[4] = "", yawStr[4] = "", pitchStr[4] = "", rollStr[4] = "";
int rcInput[4] = {0};
float inputKp, inputKi, inputKd;

//App input integer value buffer to check for failsafe
int inputBuffer[4] = {0,0,0,0};

//Final Input Values
int yawInput, pitchInput, rollInput, thrInput;

byte userConnected = 0;



//----------FAILSAFE----------//

#define LAND_RATE 1
#define NO_PACKET_LIMIT 150

unsigned int failsafeCounter = 0;
byte longFailsafeTriggered = 0;
unsigned int noPacketCounter = 0;

