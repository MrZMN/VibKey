//Measuring two MPU6050：https://github.com/goswamikapil/Arduino-Codes-Youtube/blob/master/multiple_mpu_test/multiple_mpu_test.ino
//Regarding SD read/write: https://hackingmajenkoblog.wordpress.com/2016/03/25/fast-efficient-data-storage-on-an-arduino/
//sdFat library: https://github.com/greiman/SdFat

/*
   This program measures the accelerometer data of two MPU6050s and write them in SD file
   Data format: XYZ(of sensor1)XYZ(of sensor2)XYZ(of sensor1), etc. in binary, no spaces/line breaks
   Each iteration's increased file size = SampleSize * 12 B (6B each sensor)
   Accelerometer sampling rate ≈ 500 Hz (1k Hz / 2)
*/

#include<Wire.h>
#include <SPI.h>
#include "SdFat.h"

#define SDMOUNT   // if SD card is mounted
//#define serial    // whether PC serial port opens

/*
 * MPU6050
 */
#define sensor1 0x68  // powered by VCC
#define sensor2 0x69  // powered by ADD

/*
 * GPIO
 */
#define SyncPin 10    //trigger another Arduino to vibrate the motor

/*
 * Microcontroller communication
 */
uint8_t command;

/*
 * Vibration strategy
 */
float run_vibtime;
const uint8_t num_move = 20;

/* 
 * IO
 */
SdFat SD;
File myFile;
String Filename = "vibration.dat";

/*
 * Datalog
 */
const int buf_size = 10000;  // data for 3.3s: 10000 = 3.3s * 1000 sample/s * 3 axis/sample, each axis 2B
int16_t buf[buf_size];   
long int t1, t2;    // for code execution time measurement
uint16_t buf_idx = 0;   // the size of filled buffer
int16_t endCode[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // to indicate the end of a run

/*
 * Debugging
 */
uint16_t sample_idx;    // to calculate actual ODR
uint16_t odr;


void setup() {

    /*
     * LED
     */
    pinMode(22, OUTPUT);  // red LED
    LED(22, false);
    pinMode(23, OUTPUT);  // green LED
    LED(23, false);

    /*
     * init GPIO
     */
    pinMode(SyncPin,OUTPUT);
    digitalWrite(SyncPin,LOW);

    /*
     * Serial
     */
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial1.setTimeout(2); //ms
    #ifdef serial
      while (!Serial) delay(10);  
    #endif  

    /*
     * IMU init
     */
    Wire.begin();
    imuInit(sensor1);
    imuInit(sensor2);
  
    /*
     * SD card init
     */
    #ifdef SDMOUNT
      
      if (!SD.begin(4)) {   // init SD card
        Serial.println("Card failed, or not present");
        LED(22, true);  // red LED on
        while (1);
      }
    
      //Remove the former file?
//      SD.remove(Filename);
    #endif  

    delay(1000);
}


void loop() {

    // wait for command from vibration controller
    if (Serial1.available() > 0) {
      command = Serial1.parseInt();
      Serial.println("Command: " + String(command));
      LED(22, false);  // red LED off (in case the last run collapsed)

      updateStrategy(command);  // update strategy (vib time of each move)
      
      datalog();
    }
}

/*
 * datalog during one run (i.e., a complete pairing operation)
 */
void datalog() {

    //Open file
    myFile = SD.open(Filename, FILE_WRITE);
    if(myFile) {
    
      LED(23, true);  // green LED on
      
      //for each move
      for(int j = 0; j < num_move; j++){
  
          //tell another Arduino to start the motor
          triggerArduino();
      
          //Prepare (2300 ms)
          delay(2300);  
  
          sample_idx = 0;
  
          t1 = millis();
          t2 = t1;
  
          while ((t2 - t1) < run_vibtime) { 
          
            getACC(sensor1);
            getACC(sensor2);
  
            sample_idx ++;
  
            t2 = millis();
          }
            
          odr = sample_idx/(run_vibtime/1000);  // ODR in Hz
          Serial.println("The actual ODR: " + String(odr));

          // collapsed move
          if (odr > 260 || odr < 240){  
            LED(22, true);      // red LED on
//            endCode[5] = 0xFF;  // 6 * 0xFF indicates error
//            myFile.write((const uint8_t *)&endCode, sizeof(endCode));
            imuInit(sensor1);
            imuInit(sensor2);
            j --;
          }
          // legitimate move
          else {  
            myFile.write((const uint8_t *)&buf, run_vibtime * 3);   // in bytes: time (ms) * 500 (sps) * 6 (B) / 1000
          }

          buf_idx = 0;  // reset buffer
      
          //Detach (700 ms)
          delay(700);
      }
      
      //Close file
      endCode[5] = (int16_t)command;
      myFile.write((const uint8_t *)&endCode, sizeof(endCode));
      myFile.close();
      delay(100);
      LED(23, false);  // green LED off
    }else {
      LED(22, true);  // red LED on
    }
}

/*
 * sensor reading
 */
void getACC(int sensor) {

    Wire.beginTransmission(sensor);
    Wire.write(0x3B);               // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
//    Wire.beginTransmission(sensor);
    Wire.requestFrom(sensor, 6, true); // request a total of 6 registers

    if (buf_idx < buf_size){  // no buffer overflow
      buf[buf_idx] = Wire.read() << 8 | Wire.read();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      buf[buf_idx+1] = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      buf[buf_idx+2] = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      buf_idx += 3;
    }
    delay(1);
}

/*
 * synchronise
 */
void triggerArduino(){
    digitalWrite(SyncPin,HIGH);
    delay(10); // short delay
    digitalWrite(SyncPin,LOW);
}

/*
 * init IMU
 */
void imuInit(int sensor) {

  Wire.beginTransmission(sensor);
  Wire.write(0x6B);         // PWR_MGMT_1 register
  Wire.write(0b10001000);   // DEVICE_RESET, TEMP_DIS.
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(sensor);   // Device address.
  Wire.write(0x68);                 // SIGNAL_PATH_RESET register.
  Wire.write(0b00000111);           // GYRO_RESET, ACCEL_RESET, TEMP_RESET.
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(sensor);
  Wire.write(0x6B);         // PWR_MGMT_1 register
  Wire.write(0b00001000);   // SLEEP = 0, TEMP_DIS = 1.
  Wire.endTransmission();

  Wire.beginTransmission(sensor);    
  Wire.write(0x1C);                     
  Wire.write(0b00010000);               //Setting the accel to +/- 8g                                                                   
  Wire.endTransmission();
}

/*
 * Vibration strategy
 */
void updateStrategy(uint8_t command){
  switch(command) {
    case 0x00:
      run_vibtime = 1750;
      break;
    case 0x20:
      run_vibtime = 1200; // 3 * 400 
      break;  
    /*
     * Here we replace 600 and 800 to 700, in a way of minimal changes.
     */   
    case 0x24:
      //run_vibtime = 1800; // 3 * 600 
      run_vibtime = 2100; // 3 * 700 
      break;    
    case 0x28:
      run_vibtime = 2400; // 3 * 800 
      break;
    case 0x2C:
      run_vibtime = 3000; // 3 * 1000 
      break;      
    case 0x10:
      run_vibtime = 400;
      break;    
    case 0x11:
      run_vibtime = 400;
      break;   
    case 0x12:
      run_vibtime = 400;
      break;   
    /*
     * Here we replace 600 and 800 to 700, in a way of minimal changes.
     */ 
    case 0x14:
      //run_vibtime = 600;
      run_vibtime = 700;
      break;   
    case 0x15:
      //run_vibtime = 600;
      run_vibtime = 700;
      break;   
    case 0x16:
      //run_vibtime = 600;
      run_vibtime = 700;
      break;   
    case 0x18:
      run_vibtime = 800;
      break;   
    case 0x19:
      run_vibtime = 800;
      break;   
    case 0x1A:
      run_vibtime = 800;
      break;   
    case 0x1C:
      run_vibtime = 1000;
      break;  
    case 0x1D:
      run_vibtime = 1000;
      break;         
    case 0x1E:
      run_vibtime = 1000;
      break;      
    default:
      Serial.println("Malicious command");
  }
}

/*
 * LED glow
 * LED_id == 22, red
 * LED_id == 23, green
 */
void LED(int LED_id, boolean ledswitch) {
  if(ledswitch == true) {
    digitalWrite(LED_id, LOW);  // LED on
  }else {
    digitalWrite(LED_id, HIGH);
  }
}
