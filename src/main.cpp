#include <Arduino.h>
#include <Wire.h>

#define DEBUG 1
#define TIMER_PRESCALER 80
#define TIMER_COUNTER_UP true
#define TIMER_COUNTER_DELAY 100000   // (Clock/Prescaler) counted units
#define TIMER_AUTO_RELOAD true
#define MPU_ADDR (uint16_t)0x68
#define CONTINUE_COMM (bool)true
#define I2C_CLOCK 400000L
#define PWR_MGMT_1 0x6B             // Power management register 1
#define PWR_MGMT_2 0x6C             // Power management register 2
#define SMPRT_DIV 0x19              // Sample Rate divider register
#define CONFIG 0x1A                 // Config register
#define ACCEL_CONFIG 0x1C           // Accelerometer Configuration Register
#define FIFO_EN 0x23                // FIFO Enable register
#define INT_ENABLE 0x38             // INT Enable register
#define INT_STATUS 0x3A             // INT Status register
#define FIFO_OFLOW_INT 4            // FIFO overvlow status bit
#define ACCEL_ZOUT_H 0x3F           // High bits from last read accelerometer value
#define ACCEL_ZOUT_L 0x40           // Low bits from last read accelerometer value
#define USER_CTRL 0x6A              // User control register
#define FIFO_COUNT_H 0x72           // High bits from FIFO counter
#define FIFO_COUNT_L 0x73           // Low bits from FIFO counter
#define FIFO_R_W 0x74               // FIFO Read and Write register


//Timer Variables
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Timer function
void IRAM_ATTR onTimer(void) {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

int16_t FIFO_count(void){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(FIFO_COUNT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (int8_t)2, CONTINUE_COMM);
  return (int16_t) (Wire.read() << 8 | Wire.read());
}

bool FIFO_overflow(void){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (int8_t)1, CONTINUE_COMM);
  return (bool) ((1 << FIFO_OFLOW_INT) & Wire.read());
}

void FIFO_read(int16_t count, int16_t* values){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(FIFO_R_W);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (int8_t)count, CONTINUE_COMM);
  for(int16_t i=0; i<count; i+=2){
    values[(i+1)/2] = (int16_t) (Wire.read() << 8 | Wire.read());
  }
}

//Accelerometer configuration
void mpu_config(void){
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  Wire.beginTransmission(MPU_ADDR);
  byte config;
  
  config = 0;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(config);
  Wire.endTransmission(true);
  delay(100);
  
  Wire.beginTransmission(MPU_ADDR);
  config |= (1 << 7);                         //Reset Device Config (1) reset
  Wire.write(PWR_MGMT_1);
  Wire.write(config);
  Wire.endTransmission(true);

  delay(1000);

  config = 0;
  config &= (0 << 0) && (0 << 1) && (0 << 2); //Clock Selection: (0) Internal
  config |= (1 << 3);                         //Disable Temp sensor
  config &= (0 << 4);                         // ----
  config &= (0 << 5);                         // Cycle: (0) Disabled
  config &= (0 << 6);                         // Sleep: (0) Wake
  config &= (0 << 7);                         //Reset Device Config (0) on
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0;
  config |= (1 << 0);                         // Gyro Z Standby: (1) Standby
  config |= (1 << 1);                         // Gyro Y Standby: (1) Standby
  config |= (1 << 2);                         // Gyro X Standby: (1) Standby
  config &= (0 << 3);                         // Accel Z Standby: (0) Working
  config |= (1 << 4);                         // Accel Y Standby: (1) Standby
  config |= (1 << 5);                         // Accel X Standby: (1) Standby
  config &= (0 << 6) & (0 << 7);              // Low Power Wake Control: (not used)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_2);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0x08;                              // Sample rate divider: 8
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(SMPRT_DIV);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0;                                 // low pass filter 260Hz and Fsync deactivated
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0;
  config |= (1 << 0);                         // AFS_SEL = 1 (Scale = +-4g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0;
  config |= (1 << 3);                         // ACCEL FIFO EN = 1 (FIFO enabled for Accel)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(FIFO_EN);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0;
  config |= (1 << 0);                         // Data Ready INT: (1) enabled
  config |= (1 << 4);                         // FIFO OVERFLOW INT: (1) enabled
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_ENABLE);
  Wire.write(config);
  Wire.endTransmission(true);

  config = 0;
  config |= (1 << 2);                         // FIFO reset: (1) reset
  config |= (1 << 6);                         // FIFO buffer  operations: (1) enabled
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(USER_CTRL);
  Wire.write(config);
  Wire.endTransmission(true);
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

  mpu_config();

  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_ZOUT_H);  // starting with register 0x3B (ACCEL_ZOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2, CONTINUE_COMM);
    int16_t AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Serial.println(AcZ);
  }
}