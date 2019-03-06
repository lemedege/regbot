// MPU-9150 Accelerometer + Gyro + Compass + Temperature
// -----------------------------
//
// By arduino.cc user "frtrobotik" (Tobias Hübner)
//
//
// July 2013
//      first version
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version,
// since Wire.endTransmission() uses a parameter
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-9150 Product Specification Revision 4.0",
//     PS-MPU-9150A.pdf
//   - "MPU-9150 Register Map and Descriptions Revision 4.0",
//     RM-MPU-9150A-00.pdf
//   - "MPU-9150 9-Axis Evaluation Board User Guide"
//     AN-MPU-9150EVB-00.pdf
//
// The accuracy is 16-bits.
//
// Some parts are copied by the MPU-6050 Playground page.
// playground.arduino.cc/Main/MPU-6050
// There are more Registervalues. Here are only the most
// nessecary ones to get started with this sensor.

#include <core_pins.h>
#include "main.h"
#include "mpu9150.h"
#include "eeconfig.h"
// I2C address depends on your wiring. Set i mpu9150.h file.
static int MPU9150_I2C_ADDRESS = ADDRESS_MPU;
static int IMU_MAG_I2C_ADDR = 0x0C;


//Variables where our values can be stored
// int cmps[3] = {0,0,0};
// int accl[3] = {0,0,0};
// int gyro[3] = {0,0,0};
int gyrotemp = 8;
//float pitch;

// float cmps_x() { return cmps[0]; }
// float cmps_y() { return cmps[1]; }
// float cmps_z() { return cmps[2]; }
// int gyro_x() { return gyro[0]; }
// int gyro_y() { return gyro[1]; }
// int gyro_z() { return gyro[2]; }
// float accel_x() { return           accl[0] * 9.82 / 16384.0; }
// float accel_y() { return           accl[1] * 9.82 / 16384.0; }
// float accel_z() { return ( (float) accl[2] * 9.82 / 16384.0); }
//float tempGyro() { return ( (float)gyrotemp - 12412.0 ) / 340.0; }
// float getPitch() { return pitch; }

// data from imu
int16_t imuAcc[3]  = {0,0,0};
int16_t imuGyro[3] = {0,0,0};
int32_t offsetGyro[3] = {0,0,0};
int16_t imuMag[3]     = {0,0,0};
int16_t imuTemp;
bool    gyroOffsetDone = false;

uint32_t gyroOffsetStartCnt = 1000;

const int gyroSensitivity = 2; // 0 = 250 deg/sec, 1=500, 2=1000 3 = 2000 deg/sec
const float gyroScaleFac = 250.0 / 32768.0 * float(1 << gyroSensitivity);
int32_t gyroSteadyMaxValue = int(10.0 / gyroScaleFac); // in degrees/s
const int accSensitivity = 1;
const float accScaleFac = 2*9.82 / 32768.0 * float(1 << accSensitivity);

int16_t MPU9150_readSensor(int addr);
void MPU9150_setupCompass();
int MPU9150_writeSensor(int addr,int data);


/**
 * write one byte to a sensor (i2c) register
 * \param addr is i2c 7-bit address
 * \param reg is (8-bit) register to write to
 * \param data is 8-bit data to write
 * \returns 0 in succes and 1, 2, 3, 4 on error */
int writeSensor(uint8_t i2caddr, int reg, int data)
{
  // pack buffer
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);
  Wire.write(data);
  // send package
  return Wire.endTransmission(I2C_STOP); // send the data
}

/**
 * Request a number of data from an i2c client
 * the data is in the Wire (or Wire1) buffer and read by Wire.read() called
 * \param i2caddr is the 7-bit address of the device ox68 for imu or 0x0c for readMagnetometer
 * \param reg is register number to read from
 * \param readCnt is number of registers to read
 * \returns number of characters available in read buffer */
int readToBuffer(uint8_t i2cAddr, uint8_t reg, uint8_t readCnt)
{ //
  int a;
  //
  Wire.beginTransmission(i2cAddr);
  Wire.write(reg);
  a = Wire.endTransmission(I2C_NOSTOP,1000);
  a = Wire.requestFrom(i2cAddr, readCnt, I2C_STOP, 1000);
  return a;
}


bool MPU9150_init()
{
  int n = 0;
  //digitalWriteFast(LED_BUILTIN,LOW);
  int err;
  // Clear the 'sleep' bit to start the sensor.
  //usb_send_str("IPU init ...\r\n");
  err = MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);
//   usb_send_str("# MPU init ... done\r\n");
  if (err == 0)
  {
    // Clear the 'sleep' bit to start the sensor.
    delay(100);
    //MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);
    while (MPU9150_readSensor(MPU9150_PWR_MGMT_1) > 0x09) // Force MPU out of sleep mode
    {
      MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0x09);
      n++;
      delay(100);
      if (n > 100)
        break;
    }
    MPU9150_writeSensor(MPU9150_GYRO_CONFIG, gyroSensitivity << 3); // +/-250 o/s- 2000 o/s 
        
    MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, accSensitivity << 3); // 0 = +/-2g, 1 = +- 4g, 2 = +-8g, 3 = +- 16g
    
    // Reset of
          /*MPU9150_writeSensor(MPU9150_INT_PIN_CFG, 0x00);
          MPU9150_writeSensor(MPU9150_USER_CTRL, 0x00);
          MPU9150_writeSensor(MPU9150_GYRO_CONFIG, 0x00);
          MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, 0x00);
          MPU9150_writeSensor(MPU9150_SMPLRT_DIV, 0x00);
          MPU9150_readSensor(0x02);*/

    MPU9150_setupCompass();
    // why this?  
    writeSensor(IMU_MAG_I2C_ADDR, 0x0A, 0x01);
  }
  return err == 0;
}

// void MPU9150_loop()
// {
//   // Print all sensor values which the sensor provides
//   // Formated all values as x, y, and z in order for
//   // Compass, Gyro, Acceleration. The First value is
//   // the temperature.
//   if (Wire.done()) 
//   {
//     if (0)
//     {
//       int n;
//       n = MPU9150_readSensor(MPU9150_GYRO_XOUT_L);
//       if (n >= 0)
//         gyro[0] = n;
//     }
//     else
//     {
//       // Compass
//       //cmps[0] = MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
//       //cmps[1] = MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
//       //cmps[2] = MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);
//       // Gyro
//       gyro[0] = MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H);
//       gyro[1] = MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H);
//       gyro[2] = MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H);
//       // Accelerometer
//       accl[0] = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H);
//       accl[1] = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
//       accl[2] = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);
//       // Temperature
//       gyrotemp = MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H);
// 
//       pitch = (float)MPU9150_getPitch();
//     }
//   }
// }

//http://pansenti.wordpress.com/2013/03/26/pansentis-invensense-mpu-9150-software-for-arduino-is-now-on-github/
//Thank you to pansenti for setup code.
//I will documented this one later.
void MPU9150_setupCompass(){
  MPU9150_I2C_ADDRESS = ADDRESS_COM;//change Adress to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  MPU9150_I2C_ADDRESS = ADDRESS_MPU;//change Adress to MPU

  if (0)
  {
    MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
    MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
    MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
    MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
    MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
    MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
    MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
    MPU9150_writeSensor(0x64, 0x01); //overvride register
    MPU9150_writeSensor(0x67, 0x03); //set delay rate
    MPU9150_writeSensor(0x01, 0x80);

    MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
    MPU9150_writeSensor(0x64, 0x00); //override register
    MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
    MPU9150_writeSensor(0x64, 0x01); //override register
    MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
    MPU9150_writeSensor(0x34, 0x13); //disable slv4
  }
  else
  { // disable all secodary i2c bus operations
    MPU9150_writeSensor(0x27, 0x00); //disable ic2 slave 0
    MPU9150_writeSensor(0x2A, 0x00); //disable ic2 slave 1
    MPU9150_writeSensor(0x2D, 0x00); //disable ic2 slave 2
    MPU9150_writeSensor(0x30, 0x00); //disable ic2 slave 3
    MPU9150_writeSensor(0x34, 0x13); //disable slv4
  }
  // enable interrupt when sensor data is ready
  // and set main i2c to connect to aux i2c bus (bypass mode)
  MPU9150_writeSensor(0x6A, 0x00); //disable master i2c mode to enabel bypass
  MPU9150_writeSensor(0x37, 0x12); // clear interrupt by any read and set i2c to bypass - for mag access
  MPU9150_writeSensor(0x38, 0x01); //enable interrupt line out of MPU9150
  // magnetometer single measurement mode
  // writeSensor(IMU_MAG_I2C_ADDR, 0x0A, 0x01);
}

////////////////////////////////////////////////////////////
///////// I2C functions to get easier all values ///////////
////////////////////////////////////////////////////////////
// Replaced: false =  I2C_STOP, true = I2C_NOSTOP
// int16_t MPU9150_readSensor(int addrL, int addrH)
// {
//   int n = 100;
//   Wire.beginTransmission(MPU9150_I2C_ADDRESS);
// 
//   Wire.write(addrL);
//   Wire.endTransmission(I2C_NOSTOP,10000); // Here
//   //Wire.endTransmission(I2C_NOSTOP);
//   Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, I2C_STOP,10000);
//   //Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, I2C_STOP);
// 
//   byte L = Wire.read();
// 
//   Wire.beginTransmission(MPU9150_I2C_ADDRESS);
//   Wire.write(addrH);
//   Wire.endTransmission(I2C_NOSTOP,10000);
//   //Wire.endTransmission(I2C_NOSTOP);
// 
//   Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, I2C_STOP,10000);
//   //Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, I2C_STOP);
//   while (Wire.available() < 1)
//   {
//     if (n-- == 0)
//       return -1;
//   }
//   byte H = Wire.read();
//   return (H << 8) + L;
// }

int16_t MPU9150_readSensor(int addr){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(I2C_NOSTOP, 5000);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, I2C_STOP, 5000);
  while (Wire.available() < 1);
  return Wire.read();
}

int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  return Wire.endTransmission(I2C_STOP, 5000);
}

// double MPU9150_getPitch(){
// 	double pitch = 0;
// 
// 	double g_x = (double)MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H)/(16384.0*2.0);
// 	double g_y = (double)MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H)/(16384.0*2.0);
// 	double g_z = (double)MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H)/(16384.0*2.0);
// 
// 	pitch = atan2(g_x,sqrt(pow(g_y,2)+pow(g_z,2)))*180/M_PI;
// 
// 	return pitch;
// }

/**
 * Request data starting from register 0x3B
 * Start a receice cycle to get 14 bytes of data
 * The data is available in rx buffer when Wire.done returns 1 (or Wire.finish() returns 1) */

bool mpuRequestData()
{ // request data from IMU (ACC and Gyro - starting at register 0x3B
  int reg = 0x3b;
  // remove any old data
  Wire.flush();
  // request new data
  Wire.beginTransmission(ADDRESS_MPU);
  Wire.write(reg);
  // send request as blocking function
  uint8_t e = Wire.endTransmission(I2C_NOSTOP, 1000); // Here
  if (e != 0)
  {
    switch (e)
    { // 1=data too long, 2=recv addr NACK, 3=recv data NACK, 4=other error
      case 1: usb_send_str("#i2c set reg - data too long\r\n"); break;
      case 2: usb_send_str("#i2c set reg - recv addr NACK\r\n"); break;
      case 3: usb_send_str("#i2c set reg - recv data NACK\r\n"); break;
      case 4: usb_send_str("#i2c set reg - lost arbitration\r\n"); break;
      case 5: usb_send_str("#i2c set reg - timeout\r\n"); break;
      default: usb_send_str("#i2c set reg - unknown error\r\n"); break;
    }
  }
  const int timeoutUs = 500;
  if (false)
  {
    int a = Wire.requestFrom(ADDRESS_MPU, 14, I2C_STOP, timeoutUs);
    // send data as non-blocking
    const int MSL = 80;
    char s[MSL];
    snprintf(s, MSL, "#i2c request data - got %d bytes (in %ds)\r\n", a, timeoutUs);
    usb_send_str(s);
  }
  else
  {
    //delayMicroseconds(10);
    Wire.sendRequest(ADDRESS_MPU, 14, I2C_STOP);
  }
  return e == 0;
}

/**
 * finish current RX operation (or timeout after 200us) */
bool mpuDataReady()
{
  return Wire.finish(200);
}

/**
 * Read 2 bytes from receive buffer and 
 * \return as 16 bit value */
inline int16_t readInt16()
{
  union
  {
    uint8_t b[2];
    int16_t w;
  } ad;
  ad.b[1] = Wire.read();
  ad.b[0] = Wire.read();
  return ad.w;
}

/**
 * Read IMU data - assumed to be called some time after the mpuRequestData()
 * utilizing the time for the data to be received for other purposes.
 * If the data is not ready, then this function blocks until
 * all data is received or timeout (500us).
 * \returns true if all 14 bytes are read */
int mpuReadData()
{
  int a, b = 0;
//   const int MSL = 90;
//   char s[MSL];
//   int reg = 0x3b;
//   //
//   Wire.beginTransmission(ADDRESS_MPU);
//   Wire.write(reg);
//   ak = Wire.endTransmission(I2C_NOSTOP,10000); // Here
//   a = Wire.requestFrom(ADDRESS_MPU, 14, I2C_STOP, 1000);
  a = Wire.finish(500);
//   snprintf(s, MSL, "# finish returned %d\r\n", a);
//   usb_send_str(s);
  if (a)
  {
    b = Wire.available();
//     snprintf(s, MSL, "# available returned %d\r\n", b);
//     usb_send_str(s);
  }
  if (b >= 14)
  { // data package is available - save data
    imuAcc[0]  = readInt16();
    imuAcc[1]  = readInt16();
    imuAcc[2]  = readInt16();
    imuTemp    = readInt16();
    imuGyro[0] = readInt16();  // x-axis
    imuGyro[1] = -readInt16(); // y-axis - sign to fit with right hand rotations
    imuGyro[2] = -readInt16(); // z-axis - sign to fit with right hand rotations
  }
  if (gyroOffsetDone)
  {
    imuGyro[0] -= offsetGyro[0];
    imuGyro[1] -= offsetGyro[1];
    imuGyro[2] -= offsetGyro[2];
  }
  else if (hbTimerCnt < gyroOffsetStartCnt)
  { // zero offset before 1000 summations
    offsetGyro[0] = 0;
    offsetGyro[1] = 0;
    offsetGyro[2] = 0;
  }
  else if (hbTimerCnt <= gyroOffsetStartCnt + 1000)
  { // not finished offset calculation
//     if ((abs(imuGyro[0]) > gyroSteadyMaxValue) or 
//         (abs(imuGyro[2]) > gyroSteadyMaxValue) or 
//         (abs(imuGyro[3]) > gyroSteadyMaxValue))
//     { // wait another 100ms before staring gyro offset
//       // calculation
//       gyroOffsetStartCnt = hbTimerCnt + 100;
//       if (hbTimerCnt % 100 == 0)
//         usb_send_str("gyro offset failed - not steady\r\n");
//     }
//     else
    { // summation over 1 second
      offsetGyro[0] += imuGyro[0];
      offsetGyro[1] += imuGyro[1];
      offsetGyro[2] += imuGyro[2];
    }
    if (hbTimerCnt == gyroOffsetStartCnt + 1000)
    { // set average offset
      offsetGyro[0] /= 1000;
      offsetGyro[1] /= 1000;
      offsetGyro[2] /= 1000;
      usb_send_str("gyro offset finished\r\n");
      gyroOffsetDone  = true;
    }
  }
  else
    // redo of calibrate requested
    gyroOffsetStartCnt = hbTimerCnt + 10;
  return b;
}

/////////////////////////////////////////////////

/**
 * Read IMU data */
// int readAccGyro()
// {
//   #define IMU_REG_ACC 59
//   int a,h,l,r, ak = 0;
//   int reg = 0x3b;
//   //
//   Wire.beginTransmission(ADDRESS_MPU);
//   Wire.write(reg);
//   ak = Wire.endTransmission(I2C_NOSTOP,10000); // Here
//   a = Wire.requestFrom(ADDRESS_MPU, 14, I2C_STOP, 1000);
//   r = 0;
//   if (a > 2)
//   { // data package is available - save data
//     h = Wire.read();
//     l = Wire.read();
//     if (h >= 0 and l >= 0)
//     {
//       imuAcc[0] =(h << 8) + l;
//       r++;
//       h = Wire.read();
//       l = Wire.read();
//       if (h >= 0 and l >= 0)
//       {
//         imuAcc[1] =(h << 8) + l;
//         r++;
//         h = Wire.read();
//         l = Wire.read();
//         if (h >= 0 and l >= 0)
//         {
//           imuAcc[2] =(h << 8) + l;
//           r++;
//           h = Wire.read();
//           l = Wire.read();
//           if (h >= 0 and l >= 0)
//           {
//             imuTemp =(h << 8) + l;
//             r++;
//             h = Wire.read();
//             l = Wire.read();
//             if (h >= 0 and l >= 0)
//             {
//               imuGyro[0] =(h << 8) + l;
//               r++;
//               h = Wire.read();
//               l = Wire.read();
//               if (h >= 0 and l >= 0)
//               {
//                 imuGyro[1] =(h << 8) + l;
//                 r++;
//                 h = Wire.read();
//                 l = Wire.read();
//                 if (h >= 0 and l >= 0)
//                 {
//                   imuGyro[2] = (h<<8) +l;
//                   r++;
//                 }
//               }
//             }
//           }
//         }
//       }
//     }
//   }
//   if (gyroOffsetDone)
//   {
//     imuGyro[0] -= offsetGyro[0];
//     imuGyro[1] -= offsetGyro[1];
//     imuGyro[2] -= offsetGyro[2];
//   }
//   else if (hbTimerCnt < gyroOffsetStartCnt)
//   { // zero offset before 1000 summations
//     offsetGyro[0] = 0;
//     offsetGyro[1] = 0;
//     offsetGyro[2] = 0;
//   }
//   else if (hbTimerCnt <= gyroOffsetStartCnt + 1000)
//   { // not finished offset calculation
//     //     if ((abs(imuGyro[0]) > gyroSteadyMaxValue) or 
//     //         (abs(imuGyro[2]) > gyroSteadyMaxValue) or 
//     //         (abs(imuGyro[3]) > gyroSteadyMaxValue))
//     //     { // wait another 100ms before staring gyro offset
//     //       // calculation
//     //       gyroOffsetStartCnt = hbTimerCnt + 100;
//     //       if (hbTimerCnt % 100 == 0)
//     //         usb_send_str("gyro offset failed - not steady\r\n");
//     //     }
//     //     else
//     { // summation over 1 second
//       offsetGyro[0] += imuGyro[0];
//       offsetGyro[1] += imuGyro[1];
//       offsetGyro[2] += imuGyro[2];
//     }
//     if (hbTimerCnt == gyroOffsetStartCnt + 1000)
//     { // set average offset
//       offsetGyro[0] /= 1000;
//       offsetGyro[1] /= 1000;
//       offsetGyro[2] /= 1000;
//       usb_send_str("gyro offset finished\r\n");
//       gyroOffsetDone  = true;
//     }
//   }
//   else
//     // redo of calibrate requested
//     gyroOffsetStartCnt = hbTimerCnt + 10;
//   return ak;
// }



////////////////////////////////////////////////

void eePromSaveGyroZero()
{
  eeConfig.push32(offsetGyro[0]);
  eeConfig.push32(offsetGyro[1]);
  eeConfig.push32(offsetGyro[2]);
}

void eePromLoadGyroZero()
{
  int skipCount = 4*3;
  if (not eeConfig.isStringConfig() and robotId > 0)
  {
    offsetGyro[0] = eeConfig.read32();
    offsetGyro[1] = eeConfig.read32();
    offsetGyro[2] = eeConfig.read32();
    gyroOffsetDone = true;
  }
  else
  { // just skip, leaving default settings
    eeConfig.skipAddr(skipCount);
  }
}

//////////////////////////////////////////////////////////

void readMagnetometer()
{ // must be read by the MPU9150 first, by setting up a set of registers
  // and then reading the buffer registers in MPU9150
  // - some may be set up in initializeer of MPU9150
  #define IMU_REG_MAG 00
  int a;
  bool debug = false;
  // test for data available
  a = readToBuffer(IMU_MAG_I2C_ADDR, 2, 1);
  if (a > 0 and (Wire.read() == 1))
  {
    int h, l, r;
    int magID = 7, magInfo = 7, magSt1 = 7, magst2 = 7;
    //
    
    //   Wire.beginTransmission(IMU_MAG_I2C_ADDR);
    //   Wire.write(IMU_REG_MAG);
    //   ak = Wire.endTransmission(I2C_NOSTOP,1000);
    //   a = Wire.requestFrom(IMU_MAG_I2C_ADDR, 10, I2C_STOP, 1000);
    a = readToBuffer(IMU_MAG_I2C_ADDR, 0, 10);
    r = 0;
    if (a > 2)
    { // data package is available - save data
      magID = Wire.read();
      magInfo = Wire.read();
      magSt1 = Wire.read();
      for (int i = 0; i < 3; i++)
      {
        l = Wire.read();
        h = Wire.read();
        if (h >= 0 and l >= 0)
        {
          imuMag[i] =(h << 8) + l;
          r++;
        }
      }
      magst2 = Wire.read();
    }
    if (debug)
    {
      const int MRL = 100;
      char reply[MRL];
      snprintf(reply, MRL, "mag got %d id %x info %x st1 %x mag %7d %7d %7d st2 %x",
               a, magID, magInfo, magSt1, imuMag[0], imuMag[1], imuMag[2], magst2
      );
      usb_send_str(reply);
    }
    // magnetometer single measurement mode (again)
    writeSensor(IMU_MAG_I2C_ADDR, 0x0A, 0x01);
  }
}

////////////////////////////////////////////

void sendStatusMag()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "mag %d %d %d\r\n",
           imuMag[0], imuMag[1], imuMag[2]);
  usb_send_str(reply);
}
