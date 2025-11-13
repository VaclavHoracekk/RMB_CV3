/***************************************************** 
*                 RBM - Cvičení 3                    *       
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

// Konstanty pro vypocet
#define MAGNETIC_DECLINATION_BRNO 0.095 // Radiany
#define PRESSURE_BRNO 1022.4 // hPa

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
  float altitude;
};

struct THmcData
{
  // Magnetometer
  float x;
  float y;
  float z;

  float headingDegrees;
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
  float temp_alti = 0;

  do
  {
    temp_temp = bme.readTemperature();
    temp_pres = bme.readPressure();
    temp_hum = bme.readHumidity();
    temp_alti = bme.readAltitude(PRESSURE_BRNO);

    if(temp_hum == 0 && temp_pres == 0 && temp_temp == 0 && temp_alti == 0)
      Serial.println("VAROVANI: Senzor BME280 necte validni hodnoty!");

    delay(1000);
  } while (temp_hum == 0 && temp_pres == 0 && temp_temp != 0 && temp_alti == 0);
  
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
  } while (event.magnetic.x == 0 && event.magnetic.y == 0 && event.magnetic.y == 0);
  
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
  } while (check_sum == 0);
  
  Serial.println("Senzor MPU6050 uspesne inicializovan \n");
}

struct TBmeData getBmeData(bool print_status)
{
  struct TBmeData temp_bme;

  temp_bme.temperature = bme.readTemperature();
  temp_bme.pressure = bme.readPressure();
  temp_bme.humidity = bme.readHumidity();
  temp_bme.altitude = bme.readAltitude(PRESSURE_BRNO);

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

    Serial.print("BME280 Nadmorska vyska: ");
    Serial.print(temp_bme.altitude);
    Serial.print("m");

    Serial.print("\n");
  }

  return temp_bme;
}

struct THmcData getHmcData(bool print_status)
{
  struct THmcData temp_hmc;

  sensors_event_t event; 
  hmc.getEvent(&event);

  temp_hmc.x = event.magnetic.x;
  temp_hmc.y = event.magnetic.y;
  temp_hmc.z = event.magnetic.z;

  float heading = atan2(event.magnetic.y, event.magnetic.x); // Natoceni od severu
  heading += MAGNETIC_DECLINATION_BRNO; // Kompenzace

  // Kontrola otoceni
  if(heading < 0)
    heading += 2*PI;
  
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Radiany na stupne
  temp_hmc.headingDegrees = heading * 180/M_PI; 

  if(print_status)
  {
    Serial.print("HMC5883 X: ");
    Serial.print(temp_hmc.x );
    Serial.print("uT");

    Serial.print(", ");

    Serial.print("HMC5883 Y: ");
    Serial.print(temp_hmc.y);
    Serial.print("uT");

    Serial.print(", ");

    Serial.print("HMC5883 Z: ");
    Serial.print(temp_hmc.z);
    Serial.print("uT \n");

    Serial.print("\n");

    Serial.print("HMC5883 Natoceni od severu: ");
    Serial.print(temp_hmc.headingDegrees);
    Serial.print("*");

    Serial.print("\n");
  }

  return temp_hmc;
}

struct TMpuData getMpuData(bool print_status)
{
  struct TMpuData temp_mpu;

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  temp_mpu.a_x = acc.acceleration.x;
  temp_mpu.a_y = acc.acceleration.y;
  temp_mpu.a_z = acc.acceleration.z;

  temp_mpu.gyro_x = gyro.gyro.x;
  temp_mpu.gyro_y = gyro.gyro.y;
  temp_mpu.gyro_z = gyro.gyro.z;

  temp_mpu.temp = temp.temperature;

  if(print_status)
  {
    Serial.print("MPU6050 Zrychleni X: ");
    Serial.print(temp_mpu.a_x);
    Serial.print("m/s^2");

    Serial.print(", ");

    Serial.print("MPU6050 zrychleni Y: ");
    Serial.print(temp_mpu.a_y);
    Serial.print("m/s^2");

    Serial.print(", ");

    Serial.print("MPU6050 zrychleni Z: ");
    Serial.print(temp_mpu.a_z);
    Serial.print("m/s^2");

    Serial.print("\n");

    Serial.print("MPU6050 gyroskop X: ");
    Serial.print(temp_mpu.gyro_x );
    Serial.print("rad/s");

    Serial.print(", ");

    Serial.print("MPU6050 gyroskop Y: ");
    Serial.print(temp_mpu.gyro_y);
    Serial.print("rad/s");

    Serial.print(", ");

    Serial.print("MPU6050 gyroskop Z: ");
    Serial.print(temp_mpu.gyro_z);
    Serial.print("rad/s");

    Serial.print("\n");

    Serial.print("MPU6050 Teplota: ");
    Serial.print(temp_mpu.temp);
    Serial.print("*C");

    Serial.print("\n");
  }

  return temp_mpu;
}


//------------------ Setup - priprava periferii ------------------
void setup()
{
  // Inicializace serialu
  Serial.begin(115200);

  // Kontroler ceka, dokud se serial nerozbehne
  while(!Serial)
    delay(10);

  // Inicializace senzoru
  hmcInit();
  mpuInit();
  bmeInit();  

  // Cteni casu behu programu
  long cycle_time = millis();
}

//------------------ Main loop - hlavni smycka programu ------------------
void main()
{
  // Inicializace instanci datovych struktur pro senzory
  struct THmcData hmc_data;
  struct TMpuData mpu_data;
  struct TBmeData bme_data;

  // Cteni ze senzoru s periodou PERIOD
  if(millis() >= cycle_time + PERIOD)
  {
    // Cteni casu behu programu
    long cycle_time = millis();

    hmc_data = getHmcData(true);
    mpu_data = getMpuData(true);
    bme_data = getBmeData(true);
  }

  //delay(PERIOD);  // Pokud nechceme zatezovat procesor mezi merenimi

}
