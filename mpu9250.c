/**
 *Created in Fall 2019
 *Author:
 *  Samnang Chay
 *  Reference: github.com/kriswiner & freescale.com
 */
#include "mpu9250.h"
#include "i2c0.h"
#include "uart0.h"
#include "stdio.h"
#include "miscFunction.h"
#include <string.h>
#include "tm4c123gh6pm.h"

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

bool MPU_CHECK=false, MPU_IsCalibrated=false, MPU_IsInitialized=false;
bool Mag_IsInitialized=false, Mag_IsCalibrated=false;
float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 }, magBias[3] = { 0, 0, 0 }, magScale[3] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer
float magCalibration[3] = { 0, 0, 0 }; // Factory mag calibration and mag bias

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float selfTest[6]; // holds results of gyro and accelerometer self test
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
char str[80];

char param[10]={0}, cond[10]={0};
int value=0, thld;  //threshold value is effective only in 'gt' condition in gating command
bool setThld=false;

void getAres() {
  switch (Ascale) {
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void callAccel(){
    if(!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if(!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if(!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();
    accel(accelBias);

    // Print acceleration values in g!
    putsUart0("\r\n\r\nAccel.(g):  ");
    sprintf(str, "Ax=%.2f  ", ax);
    putsUart0(str);
    sprintf(str, "Ay=%.2f  ", ay);
    putsUart0(str);
    sprintf(str, "Az=%.2f  \t", az);
    putsUart0(str);
    sprintf(str, "G-force=%.2f  \t", sqrtf(ax * ax + ay * ay + az * az));
    putsUart0(str);
}

void callGyro(){
    if(!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if(!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if(!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();
    gyro(gyroBias);
    putsUart0("\r\nGyro.(deg/s):  ");
    sprintf(str, "Gx=%.2f  ", gx);
    putsUart0(str);
    sprintf(str, "Gy=%.2f  ", gy);
    putsUart0(str);
    sprintf(str, "Gz=%.2f  ", gz);
    putsUart0(str);
}

void callCompass(){
    if(!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if(!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if(!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();
    if(!Mag_IsInitialized) Mag_IsInitialized = initAK8963(magCalibration);
    if(!Mag_IsCalibrated) Mag_IsCalibrated = magCalMPU9250(magBias, magScale, magCalibration);
    compass(magBias, magScale, magCalibration);

    putsUart0("\r\nMag.field(mG):  ");
    sprintf(str, "Mx=%.1f  ", mx); putsUart0(str);
    sprintf(str, "My=%.1f  ", my); putsUart0(str);
    sprintf(str, "Mz=%.1f  ", mz); putsUart0(str);

    float iAlpha = (180.0f / M_PI * atan2(mx, my) );
    /* quandrant compass angle to show NE/NW/SE/SW */
    if (iAlpha == 0.)
        sprintf(str, "Direction: NORTH  ");
    else if (iAlpha > 0. && iAlpha < 90.)
        sprintf(str, "Direction: NE %.2f  ", iAlpha);
    else if (iAlpha == 90.)
        sprintf(str, "Direction: EAST  ");
    else if (iAlpha > 90. && iAlpha < 180.)
        sprintf(str, "Direction: SE %.2f  ", 180 - iAlpha);
    else if (iAlpha == 180.)
        sprintf(str, "Direction: SOUTH  ");
    else if (iAlpha == -90.)
        sprintf(str, "Direction: WEST  ");
    else if (iAlpha >= -90. && iAlpha < 0.)
        sprintf(str, "Direction: NW %.2f  ", -iAlpha);
    else if (iAlpha > -180. && iAlpha < -90.)
        sprintf(str, "Direction: SW %.2f  ", (iAlpha + 180));
    putsUart0(str);

    if (!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if (!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if (!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();

    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    ax = -accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    ay = -accelCount[1] * aRes - accelBias[1];
    az = -accelCount[2] * aRes - accelBias[2];


    /* roll pitch and yaw angles computed by iecompass */
    float iPhi, iThe, iPsi;
    /* magnetic field readings corrected for hard iron effects and PCB orientation */
    float iBfx, iBfy; //, iBfz;

    /* hard iron estimate */
    int16_t iVx = 0, iVy = 0, iVz = 0;

    /* tilt-compensated e-Compass code */
    float iBpx = mx, iBpy = my, iBpz = mz, iGpx = ax, iGpy = ay, iGpz = az;
    /* subtract the hard iron offset */
    iBpx -= iVx; /* see Eq 16 */
    iBpy -= iVy; /* see Eq 16 */
    iBpz -= iVz; /* see Eq 16 */

    /* calculate current roll angle Phi */
    iPhi = atan2(iGpy, iGpz);/* Eq 13 */
    iBfy = (iBpy * cos(iPhi) - iBpz * sin(iPhi));/* Eq 19 y component */

    /* calculate current pitch angle Theta */
    iThe = atan2(-iGpx, (iGpy*sin(iPhi) + iGpz*cos(iPhi)) );/* Eq 15 */
    /* restrict pitch angle to range -90 to 90 degrees */
    if (iThe > 90)
        iThe = (180 - iThe);
    if (iThe < -90)
        iThe = (-180 - iThe);

    iBfx = (iBpx * cos(iThe) + iBpy * sin(iThe)*sin(iPhi) + iBpz * sin(iThe)*cos(iPhi) ); /* Eq 19: x component */
    iPsi = atan2(-iBfy, iBfx); /* Eq 22 */
    sprintf(str, "\r\nYaw, pitch, roll (deg): %.1f, %.1f, %.1f\r\n", 180.0f / M_PI * iPsi, 180.0f / M_PI * iThe, 180.0f / M_PI * iPhi);
    putsUart0(str);

}

bool checkMPU9250()
{
    uint8_t id = readI2c0Register(MPU9250, WHO_AM_I);
    if (id == 0x71)
    {
        putsUart0("\r\nDevice connected. Initializing");
        MPU9250SelfTest(selfTest);
    }
    else
    {
        sprintf(str, "Connection failed, id: 0x%02hhx\r\n", id);
        putsUart0(str);
        HWREG(HIB_DATA + (15 * 4)) = NULL;
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

        return false;
    }
    return true;
}

void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
    int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
    float factoryTrim[6];
    uint8_t FS = 0, ii;

    writeI2c0Register(MPU9250, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
    writeI2c0Register(MPU9250, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeI2c0Register(MPU9250, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
    writeI2c0Register(MPU9250, ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeI2c0Register(MPU9250, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

    for (ii = 0; ii < 200; ii++) {  // get average current values of gyro and accelerometer
        readI2c0Registers(MPU9250, ACCEL_XOUT_H, &rawData[0], 6); // Read the six raw data registers into data array
        aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

        readI2c0Registers(MPU9250, GYRO_XOUT_H, &rawData[0], 6); // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
    }

    for (ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    writeI2c0Register(MPU9250, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeI2c0Register(MPU9250, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    waitMicrosecond(25000);  // Delay a while to let the device stabilize

    for (ii = 0; ii < 200; ii++)
    {  // get average self-test values of gyro and acclerometer
        readI2c0Registers(MPU9250, ACCEL_XOUT_H, &rawData[0], 6); // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

        readI2c0Registers(MPU9250, GYRO_XOUT_H, &rawData[0], 6); // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
    }

    for (ii = 0; ii < 3; ii++)
    {  // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeI2c0Register(MPU9250, ACCEL_CONFIG, 0x00);
    writeI2c0Register(MPU9250, GYRO_CONFIG, 0x00);
    waitMicrosecond(25000);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readI2c0Register(MPU9250, SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readI2c0Register(MPU9250, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readI2c0Register(MPU9250, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readI2c0Register(MPU9250, SELF_TEST_X_GYRO); // X-axis gyro self-test results
    selfTest[4] = readI2c0Register(MPU9250, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
    selfTest[5] = readI2c0Register(MPU9250, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] = (float) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (ii = 0; ii < 3; ii++)
    {
        destination[ii] = 100.0 * ((float) (aSTAvg[ii] - aAvg[ii])) / factoryTrim[ii];   // Report percent differences
        destination[ii + 3] = 100.0 * ((float) (gSTAvg[ii] - gAvg[ii])) / factoryTrim[ii + 3]; // Report percent differences
    }
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
bool calibrateMPU9250(float * dest1, float * dest2) {
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

    // reset device
    writeI2c0Register(MPU9250, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    waitMicrosecond(100000);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    writeI2c0Register(MPU9250, PWR_MGMT_1, 0x01);
    writeI2c0Register(MPU9250, PWR_MGMT_2, 0x00);
    waitMicrosecond(200000);

    // Configure device for bias calculation
    writeI2c0Register(MPU9250, INT_ENABLE, 0x00);   // Disable all interrupts
    writeI2c0Register(MPU9250, FIFO_EN, 0x00);      // Disable FIFO
    writeI2c0Register(MPU9250, PWR_MGMT_1, 0x00); // Turn on internal clock source
    writeI2c0Register(MPU9250, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeI2c0Register(MPU9250, USER_CTRL, 0x00); // Disable FIFO and I2C master modes
    writeI2c0Register(MPU9250, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    waitMicrosecond(15000);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeI2c0Register(MPU9250, CONFIG, 0x01);   // Set low-pass filter to 188 Hz
    writeI2c0Register(MPU9250, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeI2c0Register(MPU9250, GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeI2c0Register(MPU9250, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeI2c0Register(MPU9250, USER_CTRL, 0x40);   // Enable FIFO
    writeI2c0Register(MPU9250, FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    waitMicrosecond(40000);

    // At end of sample accumulation, turn off FIFO sensor read
    writeI2c0Register(MPU9250, FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
    readI2c0Registers(MPU9250, FIFO_COUNTH, &data[0], 2); // read FIFO sample count
    fifo_count = ((uint16_t) data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
        readI2c0Registers(MPU9250, FIFO_R_W, &data[0], 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0] += (int32_t) gyro_temp[0];
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0] /= (int32_t) packet_count;
    gyro_bias[1] /= (int32_t) packet_count;
    gyro_bias[2] /= (int32_t) packet_count;

    if (accel_bias[2] > 0) {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    writeI2c0Register(MPU9250, data[0], XG_OFFSET_H);
    writeI2c0Register(MPU9250, data[1], XG_OFFSET_L);
    writeI2c0Register(MPU9250, data[2], YG_OFFSET_H);
    writeI2c0Register(MPU9250, data[3], YG_OFFSET_L);
    writeI2c0Register(MPU9250, data[4], ZG_OFFSET_H);
    writeI2c0Register(MPU9250, data[5], ZG_OFFSET_L);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
    dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
    readI2c0Registers(MPU9250, XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
    readI2c0Registers(MPU9250, YA_OFFSET_H, &data[0], 2);
    accel_bias_reg[1] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
    readI2c0Registers(MPU9250, ZA_OFFSET_H, &data[0], 2);
    accel_bias_reg[2] = (int32_t) (((int16_t) data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
        if ((accel_bias_reg[ii] & mask))
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    writeI2c0Register(MPU9250, XA_OFFSET_H, data[0]);
    writeI2c0Register(MPU9250, XA_OFFSET_L, data[1]);
    writeI2c0Register(MPU9250, YA_OFFSET_H, data[2]);
    writeI2c0Register(MPU9250, YA_OFFSET_L, data[3]);
    writeI2c0Register(MPU9250, ZA_OFFSET_H, data[4]);
    writeI2c0Register(MPU9250, ZA_OFFSET_L, data[5]);
    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
    dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
    dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
    return true;
}

void WoM(){
    writeI2c0Register(MPU9250, PWR_MGMT_1, (0 << 6 | 0 << 5 | 0 << 4));
    writeI2c0Register(MPU9250, PWR_MGMT_2, 0b111);
    writeI2c0Register(MPU9250, ACCEL_CONFIG_2, 0x1);
    writeI2c0Register(MPU9250, INT_ENABLE, 0x40 );
    writeI2c0Register(MPU9250, MOT_DETECT_CTRL, (1<<6 | 1<<7));
    writeI2c0Register(MPU9250, WOM_THR, 0x7F); //configured for 510mg threshold (0-1020mg)
    writeI2c0Register(MPU9250, LP_ACCEL_ODR, 0x3F); //(0.24-500Hz)
    writeI2c0Register(MPU9250, PWR_MGMT_1, 0x3);
    writeI2c0Register(MPU9250, INT_PIN_CFG, 0b11000000 );
}

bool initMPU9250() {
    writeI2c0Register(MPU9250, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    waitMicrosecond(100000); // Wait for all registers to reset
    writeI2c0Register(MPU9250, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    waitMicrosecond(200000);
    writeI2c0Register(MPU9250, CONFIG, 0x03);
    writeI2c0Register(MPU9250, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
    uint8_t c = readI2c0Register(MPU9250, GYRO_CONFIG); // get current GYRO_CONFIG register value
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    writeI2c0Register(MPU9250, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readI2c0Register(MPU9250, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    writeI2c0Register(MPU9250, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    c = readI2c0Register(MPU9250, ACCEL_CONFIG_2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeI2c0Register(MPU9250, ACCEL_CONFIG_2, c); // Write new ACCEL_CONFIG2 register value
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Tiva board as master
    writeI2c0Register(MPU9250, INT_PIN_CFG, 0x22);
    writeI2c0Register(MPU9250, INT_ENABLE, 0x1); // Enable data ready (bit 0) interrupt

    waitMicrosecond(200000);
    putsUart0("\r\n\tInitialized Accelero, gyro-meters");
    return true;
}

bool initAK8963(float * magCalibration) {
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeI2c0Register(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    writeI2c0Register(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    readI2c0Registers(AK8963_ADDRESS, AK8963_ASAX, &rawData[0], 3); // Read the x-, y-, and z-axis calibration values
    magCalibration[0] = (float) (rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] = (float) (rawData[1] - 128) / 256. + 1.;
    magCalibration[2] = (float) (rawData[2] - 128) / 256. + 1.;
    writeI2c0Register(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    writeI2c0Register(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    putsUart0("\r\n\tInitialized Magnetometer"); // Initialize device for active mode read of magnetometer
    return true;
}

bool magCalMPU9250(float * magBias, float * magScale, float * magCalibration) {
    uint16_t ii = 0, sample_count = 0;
    int16_t mag_max[3] = { 0xFF, 0xFF, 0xFF }, mag_min[3] = { 0x7F, 0x7F, 0x7F }, mag_temp[3] = { 0, 0, 0 };
    char str[80];
    putsUart0("\r\nTo calibrate the Magnetometer, wave device multiple times in figure of eight..\r\n");
    uint8_t _Mmode=0x02;
    if (_Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
    if (_Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms

    for (ii = 0; ii < sample_count; ii++)
    {
        readMagData(mag_temp);  // Read the mag data
        uint8_t jj;
        for (jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        if (Mmode == 0x02) waitMicrosecond(135000);  // at 8 Hz ODR, new mag data is available every 125 ms
        if (Mmode == 0x06) waitMicrosecond(12000);  // at 100 Hz ODR, new mag data is available every 10 ms
    }
    if (SerialDebug){
        sprintf(str, "\tmag x min/max:%u, %u\r\n", mag_max[0], mag_min[0]);
        putsUart0(str);
        sprintf(str, "\tmag y min/max:%u, %u\r\n", mag_max[1], mag_min[1]);
        putsUart0(str);
        sprintf(str, "\tmag z min/max:%u, %u\r\n", mag_max[2], mag_min[2]);
        putsUart0(str);
    }
    // Get hard iron correction
    magBias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    magBias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    magBias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    magBias[0] = (float) magBias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
    magBias[1] = (float) magBias[1] * mRes * magCalibration[1];
    magBias[2] = (float) magBias[2] * mRes * magCalibration[2];

    // Get soft iron correction estimate
    magScale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
    magScale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
    magScale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts
    float avg_rad = magScale[0] + magScale[1] + magScale[2];
    avg_rad /= 3.0;
    magScale[0] = avg_rad / ((float) magScale[0]);
    magScale[1] = avg_rad / ((float) magScale[1]);
    magScale[2] = avg_rad / ((float) magScale[2]);

    putsUart0("Calibrated.");

    return true;
}

void readMagData(int16_t * destination) {
    uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if (readI2c0Register(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readI2c0Registers(AK8963_ADDRESS, AK8963_XOUT_L, &rawData[0], 7); // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t) rawData[3] << 8) | rawData[2]; // Data stored as little Endian
            destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
        }
    }
}

void readAccelData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readI2c0Registers(MPU9250, ACCEL_XOUT_H, &rawData[0], 6);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readGyroData(int16_t * destination) {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readI2c0Registers(MPU9250, GYRO_XOUT_H, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

void readTempData() {
  uint16_t tempCount;
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readI2c0Registers(MPU9250, TEMP_OUT_H, &rawData[0], 2);  // Read the two raw data registers sequentially into data array
  tempCount = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
  sprintf(str, "Temperature (C): %.2f\r\n", ((float) tempCount) / 333.87 + 13.8);
  putsUart0(str);
}

void getMres() {
  switch (Mscale) {
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale) {
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void accel(float * accelBias){
    if (readI2c0Register(MPU9250, INT_STATUS) & 0x01)
    {
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();
        // Now we'll calculate the acceleration value into actual g's
        ax = -accelCount[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = -accelCount[1] * aRes - accelBias[1];
        az = -accelCount[2] * aRes - accelBias[2];
    }
}

void gyro(float * gyroBias){
    if (readI2c0Register(MPU9250, INT_STATUS) & 0x01)
    {
        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();
        // Calculate the gyro value into actual degrees per second
        gx = gyroCount[0] * gRes + gyroBias[0]; // get actual gyro value, this depends on scale being set
        gy = gyroCount[1] * gRes + gyroBias[1];
        gz = gyroCount[2] * gRes + gyroBias[2];
    }
}

void compass(float * magBias, float * magScale, float * magCalibration){
    if (readI2c0Register(MPU9250, INT_STATUS) & 0x01)
    {
        readMagData(magCount);  // Read the x/y/z adc values
        getMres();
        mx = (float) magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
        my = (float) magCount[1] * mRes * magCalibration[1] - magBias[1];
        mz = (float) magCount[2] * mRes * magCalibration[2] - magBias[2];

        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];
    }
}

void storeAccel(char* accels){
    if(!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if(!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if(!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();
    accel(accelBias);

    sprintf(str, "%.2f", ax);
    strcat(accels, str);
    strcat(accels, ",");
    sprintf(str, "%.2f", ay);
    strcat(accels, str);
    strcat(accels, ",");
    sprintf(str, "%.2f", az);
    strcat(accels, str);
    strcat(accels, ",");
}

void storeGyro(char* gyros){
    if(!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if(!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if(!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();
    gyro(gyroBias);

    sprintf(str, "%.2f", gx);
    strcat(gyros, str);
    strcat(gyros, ",");
    sprintf(str, "%.2f", gy);
    strcat(gyros, str);
    strcat(gyros, ",");
    sprintf(str, "%.2f", gz);
    strcat(gyros, str);
    strcat(gyros, ",");
}

void storeMag(char* mags){
    if(!MPU_CHECK) MPU_CHECK = checkMPU9250();
    if(!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
    if(!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();
    if(!Mag_IsInitialized) Mag_IsInitialized = initAK8963(magCalibration);
    if(!Mag_IsCalibrated) Mag_IsCalibrated = magCalMPU9250(magBias, magScale, magCalibration);
    compass(magBias, magScale, magCalibration);

    sprintf(str, "%.1f", mx);
    strcat(mags, str);
    strcat(mags, ",");
    sprintf(str, "%.1f", my);
    strcat(mags, str);
    strcat(mags, ",");
    sprintf(str, "%.1f", mz);
    strcat(mags, str);
    strcat(mags, ",");
}

void storeTemp(char* temps){
  uint16_t tempCount;
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readI2c0Registers(MPU9250, TEMP_OUT_H, &rawData[0], 2);  // Read the two raw data registers sequentially into data array
  tempCount = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
  float temp = ((float) tempCount) / 333.87 + 13.8;

  sprintf(str, "%.1f", temp );
  strcat(temps, str);
  strcat(temps, ",");
}

void gatingAccel(){  //To record data on triggering mode
    saveSampleData();
    waitMicrosecond(50000);

    putsUart0("\r\nAccel.(g):  ");
    sprintf(str, "Ax=%.2f  ", ax);
    putsUart0(str);
    sprintf(str, "Ay=%.2f  ", ay);
    putsUart0(str);
    sprintf(str, "Az=%.2f  \t", az);
    putsUart0(str);
    sprintf(str, "G-force=%.2f  \t", sqrtf(ax * ax + ay * ay + az * az));
    putsUart0(str);
    putsUart0("\r\n");



}

void gating_trigger(){
    if (param[0] == 'a'){
        if (!MPU_CHECK) MPU_CHECK = checkMPU9250();
        if (!MPU_IsCalibrated) MPU_IsCalibrated = calibrateMPU9250(gyroBias, accelBias);
        if (!MPU_IsInitialized) MPU_IsInitialized = initMPU9250();

        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();
        ax = -accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
        ay = -accelCount[1] * aRes - accelBias[1];
        az = -accelCount[2] * aRes - accelBias[2];

        if (strcmp(param, "ax") == 0) {
            if ((strcmp(cond, "gt") == 0) ) {
                if(!setThld) {
                    if(abs(ax*100) >= abs(value*100)) gatingAccel();
                }
                else {
                    if( abs(ax*100) <= ((abs(value)-abs(thld))*100) ) gatingAccel();
                    else if( abs(ax*100) < abs(value*100) ) putsUart0("\r\n in between, not recording");
                    else if( abs(ax*100) >= abs(value*100) ) gatingAccel();
                }
            }
            else if((strcmp(cond, "lt") == 0) && ( abs(ax*100) < abs(value*100)) ) {
                gatingAccel();
            }
        }
        else if (strcmp(param, "ay") == 0) {
            if ((strcmp(cond, "gt") == 0) ) {
                if(!setThld) {
                    if(abs(ay*100) >= abs(value*100)) gatingAccel();
                }
                else {
                    if( abs(ay*100) <= ((abs(value)-abs(thld))*100) ) gatingAccel();
                    else if( abs(ay*100) < abs(value*100) ) putsUart0("\r\n in between, not recording");
                    else if( abs(ay*100) >= abs(value*100) ) gatingAccel();
                }
            }
            else if((strcmp(cond, "lt") == 0) && ( abs(ay*100) < abs(value*100)) ) {
                gatingAccel();
            }
        }
        else if (strcmp(param, "az") == 0) {
            if ((strcmp(cond, "gt") == 0) ) {
                if(!setThld) {
                    if(abs(az*100) >= abs(value*100)) gatingAccel();
                }
                else {
                    if( abs(az*100) <= ((abs(value)-abs(thld))*100) ) gatingAccel();
                    else if( abs(az*100) < abs(value*100) ) putsUart0("\r\n in between, not recording");
                    else if( abs(az*100) >= abs(value*100) ) gatingAccel();
                }
            }
            else if((strcmp(cond, "lt") == 0) && ( abs(az*100) < abs(value*100)) ) {
                gatingAccel();
            }
        }
    }

    if (strcmp(param, "temp") == 0) {
        uint16_t tempCount;
        uint8_t rawData[2];  // x/y/z gyro register data stored here
        readI2c0Registers(MPU9250, TEMP_OUT_H, &rawData[0], 2); // Read the two raw data registers sequentially into data array
        tempCount = ((int16_t) rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a 16-bit value
        float temp = ((float) tempCount) / 333.87 + 14.8;

        if (((strcmp(cond, "gt") == 0) && (temp >= value)) || ((strcmp(cond, "lt") == 0) && (temp < value))) {
            sprintf(str, "Temperature (C): %.2f\r\n", temp);
            putsUart0(str);
        }

    }

}

void gating(char parm[], char cnd[], int val){
    strcpy(param, parm);
    strcpy(cond, cnd);
    value = val;
}

void trigger(){
    if ((HWREG(HIB_DATA + (6 * 4 )) == 1) || (HWREG(HIB_DATA + (7 * 4 )) == 1) || (HWREG(HIB_DATA + (8 * 4 )) == 1) || (HWREG(HIB_DATA + (9 * 4 )) == 1))
    {
        if ((HWREG(HIB_DATA + (14 * 4 )) == 1))
        {
            HWREG(HIB_DATA + (15 * 4)) = 't'; //mask trigger mode to wake up from motion
            while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

            WoM();
            initHibSleep(0);
        }
        else  //this is the default live mode for trigger command
        {
            if (param != 0 && cond != 0 && value != 0)
            {
                while (SW1_BTN)
                {
                    gating_trigger();
                    waitMicrosecond(200000);
                }
            }
            else
                putsUart0("Nothing to record, please check Gating parameters");
        }
    }
    else
    {
        putsUart0("Nothing to record, please check Log mask");
    }
}

void hysteresis(int threshold){  //implemented to support live mode
    if(param!=0 && cond!=0 && value!=0){
        thld = threshold;
        setThld = true;

        while(SW1_BTN){
            gating_trigger();
            waitMicrosecond(100000);
        }
        setThld = false;  //reset threshold
    }
    else
        putsUart0("Nothing to record, please check Gating parameters");
}
