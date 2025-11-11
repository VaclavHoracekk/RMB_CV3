/***************************************************** 
*               RBM - Cvičení 3                      *       
*      Komunikace se senzory BME, HMC a MPU          *
*         skrze komunikační protokol I2C             *
*                                                    *                             
*       Authors: Václav Horáček, Jan Holík           *   
******************************************************/

//------------------ Import knihoven ------------------
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

//------------------ Uzivatelska makra ------------------

// I2C adresy
#define BME_ADDRESS BME280_ADDRESS
#define MPU_ADRESS MPU6050_I2CADDR_DEFAULT                                                \
#define HMC_ADDRESS HMC5883_ADDRESS_MAG

// Konfigurace MPU
#define MPU_ACC_RANGE MPU6050_RANGE_2_G
#define MPU_GYRO_RANGE MPU6050_RANGE_250_DEG
#define MPU_FILTER_CONFIG MPU6050_BAND_21_HZ

//------------------ Uzivatelska promenne ------------------ 

// Objekty trid senzorovych class 
Adafruit_BME280 bme;
Adafruit_MPU6050 mpe;
Adafruit_HMC5883_Unified hmc;


//------------------ Uzivatelske datove typy ------------------
struct TBmeData
{
  float temperature;
  float humidity;
  float pressure;
};

struct THmcData
{
  // Magnetometer
  float x;
  float y;
  float z;
};

struct TMpuData
{
  // Accelerometer
  float a_x;
  float a_y;
  float a_z;

  // Gyroscope
  float gyro_x;
  float gyro_y;
  float gyro_z;
  
  // Calibration temperature
  float temp;
};

//------------------ Uzivatelske funkce ------------------

// Inicializuje a testuje BME280
void bmeInit()
{

}

// Inicializuje a testuje HMC5883
void hmcInit()
{

}

// Inicializuje a testuje MPU6050
void mpuInit()
{

}

struct TBmeData getBmeData()
{

}

struct THmcData getHmcData()
{
  
}

struct TMpuData getMpuData()
{
  
}


//------------------ Setup - priprava periferii ------------------
void setup()
{

}

//------------------ Main loop - hlavni smycka programu ------------------
void main()
{

}