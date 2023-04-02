#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <GOTPIDLoop.h>

#include <DynamixelShield.h>


#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(9, 10); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID = 254;
const float DXL_PROTOCOL_VERSION = 1.0;
//-------------PID---------------- 
GOTPIDLoop pid(1000); // every 1000 milliseconds

float set_vel;
float pitch;
float roll;

DynamixelShield dxl;

//IMU
NineAxesMotion mySensor;         //Object that for the sensor 
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  //dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  //PID
  pid.setTuningParameters(2, 1, 0, false); 
  pid.setDeadband(0);
  pid.setOutputLimited(0, 100);  
  pid.reset();  
  pid.setpoint = 35;

  //IMU
  //Peripheral Initialization
  Serial.begin(1000000);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor

}

void loop() {
  //IMU
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status

    Serial.print("Time: ");
    Serial.print(lastStreamTime);
    Serial.print("ms ");

    Serial.print(" H: ");
    Serial.print(mySensor.readEulerHeading()); //Heading data
    Serial.print("deg ");

    Serial.print(" R: ");
    Serial.print(mySensor.readEulerRoll()); //Roll data
    roll=mySensor.readEulerRoll();
    Serial.print("deg");

    Serial.print(" P: ");
    Serial.print(mySensor.readEulerPitch()); //Pitch data
    pitch=mySensor.readEulerPitch();
    Serial.print("deg ");

    Serial.print(" A: ");
    Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)

    Serial.print(" M: ");
    Serial.print(mySensor.readMagCalibStatus());    //Magnetometer Calibration Status (0 - 3)

    Serial.print(" G: ");
    Serial.print(mySensor.readGyroCalibStatus());   //Gyroscope Calibration Status (0 - 3)

    Serial.print(" S: ");
    Serial.print(mySensor.readSystemCalibStatus());   //System Calibration Status (0 - 3)

    Serial.println();
  }
  
  
  //Motor
  //DEBUG_SERIAL.print("Present Velocity(raw) : ");
  //DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
  //PID
  pid.processVariable = roll;//analogRead(analogInPin); 
  
  Serial.print("SP:");
  Serial.print(pid.setpoint);
  Serial.print("\tPV:");
  Serial.print(pid.processVariable);

  pid.execute();

  Serial.print("\tset vel:");
  set_vel=pid.output*0.5;
  Serial.println(set_vel);
  
  // Set Goal Velocity using RPM
  //dxl.setGoalVelocity(DXL_ID, set_vel, UNIT_PERCENT);
  
  dxl.setGoalPosition(DXL_ID, 100, UNIT_DEGREE);
  //DEBUG_SERIAL.print("Present Velocity(rpm) : ");
  //DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID, UNIT_RPM));
 

 
}
