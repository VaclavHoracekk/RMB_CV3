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
#define MPU_ADRESS MPU6050_I2CADDR_DEFAULT      
#define HMC_ADDRESS HMC5883_ADDRESS_MAG

// Konfigurace MPU
#define MPU_ACC_RANGE MPU6050_RANGE_2_G
#define MPU_GYRO_RANGE MPU6050_RANGE_250_DEG
#define MPU_FILTER_BANDWIDTH MPU6050_BAND_21_HZ

// Perioda mereni [ms]
#define PERIOD 1000

//------------------ Uzivatelska promenne ------------------ 

// Objekty trid senzorovych class 
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
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
  Serial.println("Inicializace BME280");
  unsigned status = bme.begin(BME_ADDRESS);

  while(!status)
  {
    Serial.print("VAROVANI: Senzor BME280 nebyl nelezen na adrese: ");
    Serial.print(BME_ADDRESS);
    Serial.print('\n');
    Serial.println("Zkontroluj adresu senzoru, zapojeni a hardware!");
    delay(1000);
  }

  float temp_temp = 0;
  float temp_hum = 0;
  float temp_pres = 0;

  do
  {
    temp_temp = bme.readTemperature();
    temp_pres = bme.readPressure();
    temp_hum = bme.readHumidity();

    if(temp_hum == 0 && temp_pres == 0 && temp_temp == 0)
      Serial.println("VAROVANI: Senzor BME280 necte validni hodnoty!");

    delay(1000);
  } while (temp_hum != 0 && temp_pres != 0 && temp_temp != 0);
  
  Serial.println("Senzor BME280 uspesne inicializovan \n");
}

// Inicializuje a testuje HMC5883
void hmcInit()
{
  Serial.println("Inicializace HMC5883");
  unsigned status = hmc.begin();
  while(!status)
  {
    Serial.print("VAROVANI: Senzor HMC5883 nebyl nelezen na adrese: ");
    Serial.print(HMC_ADDRESS);
    Serial.print('\n');
    Serial.println("Zkontroluj adresu senzoru, zapojeni a hardware!");
    delay(1000);
  }

  sensors_event_t event; 

  do
  {
    hmc.getEvent(&event);

    if(event.magnetic.x == 0 && event.magnetic.y == 0 && event.magnetic.z == 0)
      Serial.println("VAROVANI: Senzor HMC5883 necte validni hodnoty!");

    delay(1000);
  } while (event.magnetic.x != 0 && event.magnetic.y != 0 && event.magnetic.y != 0);
  
  Serial.println("Senzor HMC5883 uspesne inicializovan \n");
}

// Inicializuje a testuje MPU6050
void mpuInit()
{
  Serial.println("Inicializace MPU6050");
  unsigned status = mpu.begin(MPU_ADRESS);

  while(!status)
  {
    Serial.print("VAROVANI: Senzor MPU6050 nebyl nelezen na adrese: ");
    Serial.print(MPU_ADRESS);
    Serial.print('\n');
    Serial.println("Zkontroluj adresu senzoru, zapojeni a hardware!");
    delay(1000);
  }

  mpu.setAccelerometerRange(MPU_ACC_RANGE);
  mpu.setGyroRange(MPU_GYRO_RANGE);
  mpu.setFilterBandwidth(MPU_FILTER_BANDWIDTH);

  float temp_temp = 0;
  float temp_hum = 0;
  float temp_pres = 0;

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  float check_sum = 0;

  do
  {
    mpu.getEvent(&acc, &gyro, &temp);

    check_sum = acc.acceleration.x + acc.acceleration.y + acc.acceleration.z + gyro.gyro.x + gyro.gyro.y + gyro.gyro.z + temp.temperature;

    if(check_sum == 0)
      Serial.println("VAROVANI: Senzor MPU6050 necte validni hodnoty!");

    delay(1000);
  } while (check_sum != 0);
  
  Serial.println("Senzor MPU6050 uspesne inicializovan \n");
}

struct TBmeData getBmeData(bool print_status)
{
  struct TBmeData temp_bme;

  temp_bme.temperature = bme.readTemperature();
  temp_bme.pressure = bme.readPressure();
  temp_bme.humidity = bme.readHumidity();

  if (print_status)
  {
    Serial.print("BME280 Teplota: ");
    Serial.print(temp_bme.temperature);
    Serial.print("*C");

    Serial.print("\n");

    Serial.print("BME280 Tlak: ");
    Serial.print(temp_bme.pressure/100.F);
    Serial.print("hPa");

    Serial.print("\n");

    Serial.print("BME280 Vlhkost: ");
    Serial.print(temp_bme.humidity);
    Serial.print("%");

    Serial.print("\n");
  }

  return temp_bme;
}

struct THmcData getHmcData(bool print_status)
{
  
}

struct TMpuData getMpuData(bool print_status)
{
  
}


//------------------ Setup - priprava periferii ------------------
void setup()
{
  Serial.begin(9600);
}

//------------------ Main loop - hlavni smycka programu ------------------
void main()
{
  long cycleTime = millis();


  if(cycleTime >= PERIOD)
  {

    cycleTime = 0;
  }

  //delay(PERIOD);  // Pokud nechceme zatezovat procesor mezi merenimi

}