#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DEBUG 1
#define TIMER_PRESCALER 80
#define TIMER_COUNTER_UP true
#define TIMER_COUNTER_DELAY 1000000  // (Clock/Prescaler) counted units
#define TIMER_AUTO_RELOAD true

//Timer Variables
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Timer function
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

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

  //Timer Inicialization
  timer = timerBegin(0, TIMER_PRESCALER, TIMER_COUNTER_UP);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_COUNTER_DELAY, TIMER_AUTO_RELOAD);
  timerAlarmEnable(timer);

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
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.print(a.acceleration.z);
    Serial.print(",");
    Serial.print(temp.temperature);
    Serial.print(",");
    Serial.print(totalInterruptCounter);
    Serial.println();
  }
}