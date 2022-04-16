#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DEBUG

//Accelerometer elemet
Adafruit_MPU6050 mpu;

//Accelerometer configuration
void mpu_config(void){
  mpu.setGyroStandby(true,true,true);           //every gyro axis off
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerStandby(true,true,false); //Only Z accelerometer axis on
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  //mpu.setTemperatureStandby(true);              //temperature sensor off

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu.setInterruptPinPolarity(false);           //High when active
  mpu.enableCycle(false);                       //Don't enable cycle measurements
  mpu.enableSleep(false);                       //Don't enable sleep
  //mpu.setSampleRateDivisor();
  //mpu.setClock();
  //mpu.setFsyncSampleOutput();
  //mpu.setInterruptPinLatch();

#ifdef DEBUG
  Serial.print("Sample Divisor:");
  Serial.println(mpu.getSampleRateDivisor());
  Serial.print("Clock:");
  Serial.println(mpu.getClock());
  Serial.print("FSync:");
  Serial.println(mpu.getFsyncSampleOutput());
#endif
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(a.acceleration.z);
  Serial.print(",");
  Serial.println(temp.temperature);

  Serial.println("");
  delay(1);
}