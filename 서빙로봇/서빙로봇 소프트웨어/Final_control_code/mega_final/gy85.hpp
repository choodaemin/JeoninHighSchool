#ifndef _GY85IMU_HPP_
#define _GY85IMU_HPP_

#include "Wire.h"
#include<ros.h>
#include <geometry_msgs/Vector3.h>

/** ITG3205 config **/
#define GYRO_X_AXIS 1
#define GYRO_Y_AXIS 0
#define GYRO_Z_AXIS 2
#define GYRO_X_INVERT 1
#define GYRO_Y_INVERT 1
#define GYRO_Z_INVERT 1

#define ITG3205_GYRO_ADDRESS 0x68
#define ITG3205_WHO_AM_I 0x00
#define ITG3205_PWR_MGM 0x3E
#define ITG3205_RESET 0x80
#define ITG3205_DLPF_FS 0x16
#define ITG3205_SCALE 0.00121414209  //rad/s

/** ADXL345 config**/
#define ACC_X_AXIS 0
#define ACC_Y_AXIS 1
#define ACC_Z_AXIS 2
#define ACC_X_INVERT 1
#define ACC_Y_INVERT 1
#define ACC_Z_INVERT 1

#define ADXL345_DEVID 0x00
#define ADXL345_BW_RATE 0x2C
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_ACCELEROMETER_ADDRESS 0x53
#define ADXL345_DEVICE_ID 0xE5
#define ADXL345_SCALE 25.6000

/** HMC5883L config**/
#define MAG_X_AXIS 0
#define MAG_Y_AXIS 2
#define MAG_Z_AXIS 1
#define MAG_X_INVERT 1
#define MAG_Y_INVERT 1
#define MAG_Z_INVERT 1

#define HMC5883L_MAG_ADDRESS 0x1E
#define HMC5883L_MAG_ID 0x10
#define HMC5883L_MAG_REG_A 0x00
#define HMC5883L_MAG_REG_B 0x01
#define HMC5883L_MAG_MODE 0x02
#define HMC5883L_MAG_DATAX0 0x03
#define HMC5883L_MAG_GAIN 0x20  //Default gain
#define HMC5883L_MAG_SCALE 0.92  // mG/LSb


typedef struct {
	float x;
	float y;
	float z;
} Vector;

class Gy85 {
	public:
    Gy85(){};
		void init(){
      Wire.begin();
		};
		void send_value(int dev_addr, uint8_t value){
      Wire.beginTransmission(dev_addr);
      Wire.write(value);
      Wire.endTransmission();
		};
		uint8_t check_id(int dev_addr, uint8_t reg_addr){
      Wire.beginTransmission(dev_addr);
      Wire.write(reg_addr);
      Wire.endTransmission();
      delay(20);
      Wire.requestFrom(dev_addr, 1);
      return Wire.read();
		};
		void write_to_register(int dev_addr, uint8_t reg_addr, uint8_t reg_value){
      Wire.beginTransmission(dev_addr);
      Wire.write(reg_addr);
      Wire.write(reg_value);
      Wire.endTransmission();
		};

		bool check_gyroscope(){
		
      if((check_id(ITG3205_GYRO_ADDRESS, ITG3205_WHO_AM_I) & 0x7E) == ITG3205_GYRO_ADDRESS){
        write_to_register(ITG3205_GYRO_ADDRESS, ITG3205_PWR_MGM, ITG3205_RESET);
        delay(5);
        write_to_register(ITG3205_GYRO_ADDRESS, ITG3205_DLPF_FS, 0x1B);
        delay(5);
        write_to_register(ITG3205_GYRO_ADDRESS, 0x15, 0x13);
        delay(5);
        write_to_register(ITG3205_GYRO_ADDRESS, ITG3205_PWR_MGM, 0x03);
        delay(10);
        return true;
    } else {
      return false;
		  }
		};
		void measure_gyroscope(){
      uint8_t gyro_read = 0;
      send_value(ITG3205_GYRO_ADDRESS, 0x1D);
      Wire.requestFrom(ITG3205_GYRO_ADDRESS, 6);
      while(Wire.available()){
        gyro_buffer[gyro_read] = Wire.read();
        gyro_read++;
      }
      raw_rotation.x = (float)(GYRO_X_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_X_AXIS] <<8) | gyro_buffer[2*GYRO_X_AXIS+1])) * ITG3205_SCALE;  //rad/s
      raw_rotation.y = (float)(GYRO_Y_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Y_AXIS] <<8) | gyro_buffer[2*GYRO_Y_AXIS+1])) * ITG3205_SCALE;
      raw_rotation.z = (float)(GYRO_Z_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Z_AXIS] <<8) | gyro_buffer[2*GYRO_Z_AXIS+1])) * ITG3205_SCALE;
		};

		bool check_accelerometer(){
    
      if (check_id(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_DEVID) == ADXL345_DEVICE_ID){
        write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_POWER_CTL,0x08);  //D3, enables measuring
        delay(5);
        write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_DATA_FORMAT,0x09); //D3 and D0, enables FULL_RES and +/-4g   
        delay(5);
        write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_BW_RATE,0x09); //Set the bw to 0 Hz
        delay(5);
        return true;
      }
    else
      return false;
		};
		void measure_acceleration(){
      
      uint8_t acc_reads = 0;
      send_value(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_DATAX0);
      Wire.requestFrom(ADXL345_ACCELEROMETER_ADDRESS, 6);
      while(Wire.available()){
        acc_buffer[acc_reads] = Wire.read();
        acc_reads++;
      }

      raw_acceleration.x =  ((float)ACC_X_INVERT*(int16_t)((int)acc_buffer[2*ACC_X_AXIS+1]<<8 | acc_buffer[2*ACC_X_AXIS]) / ADXL345_SCALE);
      raw_acceleration.y =  ((float)ACC_Y_INVERT*(int16_t)((int)acc_buffer[2*ACC_Y_AXIS+1]<<8 | acc_buffer[2*ACC_Y_AXIS]) / ADXL345_SCALE);
      raw_acceleration.z =  ((float)ACC_Z_INVERT*(int16_t)((int)acc_buffer[2*ACC_Z_AXIS+1]<<8 | acc_buffer[2*ACC_Z_AXIS]) / ADXL345_SCALE);
		};

    bool check_magnetometer()
{
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,HMC5883L_MAG_GAIN);  //Sets the gain
  delay(5);
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18); //75Hz output
  delay(5);
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01); //Single-Measurement Mode
  delay(5);
  return true;;
}

void measure_magnetometer()
{
  uint8_t mag_reads = 0;
  send_value(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_DATAX0);
  Wire.requestFrom(HMC5883L_MAG_ADDRESS,6);
  while(Wire.available()){
    mag_buffer[mag_reads] = Wire.read();
    mag_reads++;
  }
  raw_magnetic_field.x =  (float)(MAG_X_INVERT * ((int16_t)((int)mag_buffer[2*MAG_X_AXIS] << 8) | (mag_buffer[2*MAG_X_AXIS+1]))) * HMC5883L_MAG_SCALE;
  raw_magnetic_field.y =  (float)(MAG_Y_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Y_AXIS] << 8) | (mag_buffer[2*MAG_Y_AXIS+1]))) * HMC5883L_MAG_SCALE;
  raw_magnetic_field.z =  (float)(MAG_Z_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Z_AXIS] << 8) | (mag_buffer[2*MAG_Z_AXIS+1]))) * HMC5883L_MAG_SCALE;
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);
}

    
		geometry_msgs::Vector3 raw_acceleration, raw_rotation,raw_magnetic_field;

	private:
		uint8_t gyro_buffer[6];
		uint8_t acc_buffer[6];
    uint8_t mag_buffer[6];
		TwoWire Wire;
};


#endif //_IMU_HPP_
