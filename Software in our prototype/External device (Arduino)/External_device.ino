#include <ArduinoBLE.h>

/*
 * Vibration strategy
 */
#define PWM_50_HZ 40
#define PWM_75_HZ 80
#define PWM_100_HZ 138
enum mode {
  swept = 0,
  constant,
  stepped
};
enum vibTime {  // ms
  time04 = 400,
  /*
   * Here we replace 600 and 800 to 700, in a way of minimal changes.
   */
  //time06 = 600,
  time06 = 700,
  time08 = 800,
  time10 = 1000,
  time_test = 10000
};
enum vibFreq {  // PWM val
  freq50 = PWM_50_HZ,
  freq75 = PWM_75_HZ,
  freq100 = PWM_100_HZ
};
mode run_mode;
vibTime run_vibtime;
vibFreq run_vibfreq;

/*
 * Microcontroller communication
 */
uint8_t command;
volatile uint8_t flag = 0;

/*
 * GPIO
 */
#define motorpwmPin 10  // PIN generates pwm signal feeding BJT transistor
#define SyncPin 13      // PIN receives signal from another controller
#define buzzPin 2       // PIN generates a buzz

/*
 * BLE
 */
BLEService OOBService("19B10000-E8F2-537E-4F6C-D104768A1214");
// characteristic allows remote device to read and write
BLEUnsignedCharCharacteristic OOBCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


void setup() {

    /*
     * GPIO init
     */
    pinMode(motorpwmPin,OUTPUT);
    pinMode(SyncPin,INPUT);
    pinMode(buzzPin, OUTPUT); 

    /*
     * Interrupt init
     */
    attachInterrupt(digitalPinToInterrupt(SyncPin), isr, RISING);

    /*
     * LED init
     */
    pinMode(24, OUTPUT);  // blue LED
    LED(24, false);

    /*
     * Serial
     */
    Serial.begin(115200); 
    Serial1.begin(115200);

    /*
     * BLE config
     */
    if (!BLE.begin()) {
      Serial.println("starting Bluetooth® Low Energy module failed!");
      while (1);
    }
    // set the local name peripheral advertises
    BLE.setLocalName("OOBKey");
    // set the UUID for the service this peripheral advertises
    BLE.setAdvertisedService(OOBService);
    // add the characteristic to the service
    OOBService.addCharacteristic(OOBCharacteristic);
    // add service
    BLE.addService(OOBService);
    // assign event handlers for connected, disconnected to peripheral
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
    // assign event handlers for characteristic
    OOBCharacteristic.setEventHandler(BLEWritten, OOBCharacteristicWritten);
    // set an initial value for the characteristic
    OOBCharacteristic.setValue(0);
    // start advertising
    BLE.advertise();

    delay(1000);
}

void loop() {
  // poll for Bluetooth® Low Energy events
  BLE.poll();

  // vibration (one move)
  if(flag == 1){ 
      vibrate();
      flag = 0; 
  }
}

/*
 * Vibration for a move (i.e., a 'crisp' motion)
 */
void vibrate() {

    /*
     * Prepare (2300 ms)
     */
    // Buzz (2200 ms)
    tone(buzzPin, 255);  // Hz
    delay(150);        
    noTone(buzzPin);    // Stop buzz
    delay(100);
    tone(buzzPin, 255);  // Hz
    delay(150);
    noTone(buzzPin);    // Stop buzz
    delay(1800);
    // Drive motor (100 ms)
    vib(138, 100);        // Hz, ms

    /*
     * Vibrate (datalog starts from now)
     */
    if (run_mode == swept) {
      continuousVib(10, 130, 1750);      //Sweep from 30 Hz to 125 Hz (for model 307-103)      
    } 
    else if (run_mode == constant) {
      vib(run_vibfreq, run_vibtime);
    } 
    else if (run_mode == stepped) {
      vib(PWM_50_HZ, run_vibtime);
      vib(PWM_75_HZ, run_vibtime);
      vib(PWM_100_HZ, run_vibtime);
    }
     
    analogWrite(motorpwmPin, 0);  // reset    

    /*
     * Detach (700 ms)
     */
}

/*
 * Vibration strategy
 */
void updateStrategy(uint8_t command){
  switch(command) {
    case 0x00:
      run_mode = swept;
      break;
    case 0x20:
      run_mode = stepped;
      run_vibtime = time04; 
      break;    
    case 0x24:
      run_mode = stepped;
      run_vibtime = time06;
      break;    
    case 0x28:
      run_mode = stepped;
      run_vibtime = time08;
      break;
    case 0x2C:
      run_mode = stepped;
      run_vibtime = time10;
      break;      
    case 0x10:
      run_mode = constant;
      run_vibtime = time04;
      run_vibfreq = freq50;
      break;    
    case 0x11:
      run_mode = constant;
      run_vibtime = time04;
      run_vibfreq = freq75;
      break;   
    case 0x12:
      run_mode = constant;
      run_vibtime = time04;
      run_vibfreq = freq100;
      break;   
    case 0x14:
      run_mode = constant;
      run_vibtime = time06;
      run_vibfreq = freq50;
      break;   
    case 0x15:
      run_mode = constant;
      run_vibtime = time06;
      run_vibfreq = freq75;
      break;   
    case 0x16:
      run_mode = constant;
      run_vibtime = time06;
      run_vibfreq = freq100;
      break;   
    case 0x18:
      run_mode = constant;
      run_vibtime = time08;
      run_vibfreq = freq50;
      break;   
    case 0x19:
      run_mode = constant;
      run_vibtime = time08;
      run_vibfreq = freq75;
      break;   
    case 0x1A:
      run_mode = constant;
      run_vibtime = time08;
      run_vibfreq = freq100;
      break;   
    case 0x1C:
      run_mode = constant;
      run_vibtime = time10;
      run_vibfreq = freq50;
      break;  
    case 0x1D:
      run_mode = constant;
      run_vibtime = time10;
      run_vibfreq = freq75;
      break;         
    case 0x1E:
      run_mode = constant;
      run_vibtime = time10;
      run_vibfreq = freq100;
      break; 
    /*
     * Test modes: test one single frequency for 10 s
     */
    case 0xFD:
      /*
       * Here we add a tutorial mode, with minimal changes.
       */
      run_mode = constant;
//      run_vibtime = time_test;
      run_vibtime = time06;
      run_vibfreq = freq50;
      break; 
    case 0xFE:
      run_mode = constant;
//      run_vibtime = time_test;
      run_vibtime = time10;
      run_vibfreq = freq75;
      break; 
    case 0xFF:
      run_mode = constant;
//      run_vibtime = time_test;
      run_vibtime = time10;
      run_vibfreq = freq100;
      break;      
    default:
      Serial.println("Malicious command");
  }
}

/* vibrate under a frequency (PWM value) for a time
 * volt: PWM value
 * vibTime: ms
 */
void vib(int volt, int vibTime){
    int startT = millis();
    int endT = millis();
    while((endT-startT) < vibTime){
        analogWrite(motorpwmPin, volt);
        endT = millis(); 
    }
}

/* generate a sweep vibration by linearily increasing the voltage
 * minvolt: min PWM value
 * maxvolt: max PWM value
 * vibTime: total vibration time (ms)
 */
void continuousVib(int minvolt, int maxvolt, int vibTime){
    int timefra = int(vibTime/(maxvolt-minvolt));
    for(int i=minvolt; i<maxvolt; i++){
        vib(i, timefra);
    }
}

/*
 * BLE connected
 */
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  LED(24, true);
}

/* 
 * BLE disconnected
 */
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  LED(24, false);
}

/*
 * BLE write
 */
void OOBCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  command = OOBCharacteristic.value();
  Serial.println("Command: " + String(command));

  updateStrategy(command);  // update strategy

  // send command to sensor controller (if not test mode) 
  if (command < 0xFD) {
    Serial1.print(command);
  } 
  else {  // active vibrate (if test mode)
    /*
     * Here we add a tutorial mode, with minimal changes.
     */
    for (uint8_t i = 0; i < 5; i++) {
      vibrate();
      //Detach (700 ms)
      delay(700);
    }
  }
}

/*
 * ISR
 */
void isr() {
    flag += 1;
}

/*
 * LED glow
 * LED_id == 24, blue
 */
void LED(int LED_id, boolean ledswitch) {
  if(ledswitch == true) {
    digitalWrite(LED_id, LOW);  // LED on
  }else {
    digitalWrite(LED_id, HIGH);
  }
}
