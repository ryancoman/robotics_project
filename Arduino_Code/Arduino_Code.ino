//This 

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Conditional Compilation flags - can easily adjust the data that we want out
#define SERIAL 1 // Needs to be 1 if you want the data to be printed out over a wired serial connection to be analyzed in the arduino serial monitor
#define QUAT 0
#define GYRO 1
#define ACCEL 1

BluetoothSerial SerialBT;

#define PERIPHERAL_NAME "RoboticsIMU"

MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gyro;       // [x, y, z]            gyro measurements 

double gyro_x_offset = 0.00623742;
double gyro_y_offset = -0.07095668;
double gyro_z_offset = -0.01347238;

// time
long prevTime;

//functions
void SendGyro(void);

void setup() {
  
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);
  
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  delay(10000);
  SerialBT.begin(PERIPHERAL_NAME); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  prevTime;

  Serial.println("initializing");
  mpu.initialize();
  Serial.println("done initializing");
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(54); //++
  mpu.setYGyroOffset(-21); //--
  mpu.setZGyroOffset(5);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Error
    Serial.println("MPU Error!");
  } 
}


void loop() {
#if QUAT // Send quaternion data
  int  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check if overflow
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize); //this is where we actually get the data, the data is just written to this buffer in specific locations (which dmpGetQaut... pick out)
    // above line is also what takes the longest time 
    fifoCount -= packetSize;
    long dt = micros() - prevTime;
    prevTime = micros();
    #if SERIAL
    Serial.print(dt);
    Serial.print("/");
    #endif
    SerialBT.print(dt);
    SerialBT.print("/");
    SendQuaternion();
  }
#endif
#if GYRO // Send Gyro data
  double gx;
  double gy;
  double gz;
  getGyro(&gx, &gy, &gz);
  long dt = micros() - prevTime;
  prevTime = micros();
  #if SERIAL
  Serial.print(dt);
  Serial.print("/");
  #endif
  SerialBT.print(dt);
  SerialBT.print("/");
  SendGyro(gx, gy, gz);
#endif
#if ACCEL // Send acceleration data
  SendRealAccel(); // currently sending all 0s - need to figure that one out
#endif

//Send a newline character before the next data set
#if SERIAL
Serial.println();
#endif
SerialBT.println();
}

//Helper functions

void getGyro(double *gx, double *gy, double *gz) {
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  mpu.getRotation(&gyro_x, &gyro_y, &gyro_z); // for 2000deg/s = 16.4 LSB/deg/s...for 1000deg/s = 32.8
  *gx = ((double)gyro_x / 16.4) * (PI / (double)180.0) - gyro_x_offset;
  *gy = (double) gyro_y / 16.4 * (PI / (double)180.0) - gyro_y_offset;
  *gz = (double)gyro_z / 16.4 * (PI / (double)180.0) - gyro_z_offset;// these should now be rads/sec
  
}

void SendQuaternion() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  #if SERIAL
  Serial.print(q.w, 4); Serial.print("/");
  Serial.print(q.x, 4); Serial.print("/");
  Serial.print(q.y, 4); Serial.print("/");
  Serial.print(q.z, 4); Serial.print("/");
  #endif
  
  SerialBT.print(q.w, 4); SerialBT.print("/");
  SerialBT.print(q.x, 4); SerialBT.print("/");
  SerialBT.print(q.y, 4); SerialBT.print("/");
  SerialBT.print(q.z, 4); SerialBT.print("/");
}


void SendGyro(double gx, double gy, double gz){
  #if SERIAL
  Serial.print(gx, 8); Serial.print('/');
  Serial.print(gy, 8); Serial.print('/');
  Serial.print(gz, 8); Serial.print('/');
  #endif
  SerialBT.print(gx, 8); SerialBT.print('/');
  SerialBT.print(gy, 8); SerialBT.print('/');
  SerialBT.print(gz, 8); SerialBT.print('/');
  
}

void SendRealAccel() {
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  SerialBT.print(aa.x); SerialBT.print("/");
  SerialBT.print(aa.y); SerialBT.print("/");
  SerialBT.print(aa.z); SerialBT.print("/");
}



//Below here not currently used - old code from other projects but I want to leave for now in case we want to reference later
void SendWorldAccel() {
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  Serial.print("a/");
  Serial.print(aaWorld.x); Serial.print("/");
  Serial.print(aaWorld.y); Serial.print("/");
  Serial.println(aaWorld.z);
}

void getGyroCalibration(){
  double sum_gx = 0;
  double sum_gy = 0;
  double sum_gz = 0;
  long num = 3000;
  for (int i = 0; i < num; i++){
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    mpu.getRotation(&gyro_x, &gyro_y, &gyro_z); // for 2000deg/s = 16.4 LSB/deg/s
    double gx = ((double)gyro_x / 16.4) * (PI / (double)180.0);
    double gy = (double) gyro_y / 16.4 * (PI / (double)180.0);
    double gz = (double)gyro_z / 16.4 * (PI / (double)180.0);// these should now be rads/sec
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(10); //delay 10 ms
  }
  Serial.print("Gx_avg: ");
  Serial.println(sum_gx / num, 8);
  Serial.print("Gy_avg: ");
  Serial.println(sum_gy / num, 8);
  Serial.print("Gz_avg: ");
  Serial.println(sum_gz / num, 8);

}
