/*
* This is the firmware for Prosthetic X PX-III
* It was developed for a NodeMCU board
* Initial development by Léon Spek (https://www.leonspek.nl)
*
*/



#include <Arduino.h>

#include <Servo.h>

#define SERVO_MIN 544
#define SERVO_MAX 2400


#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif




#include <ModbusIP_ESP8266.h>

#include "creds.h"
#include <WifiControl.h>
#include <ModeControl.h>

#define PX_NUM 3
#define PX_REG 100 + (PX_NUM * 10) //base operational modbus register
#define PX_LED_REG PX_REG + 1
#define PX_STATE_REG 200 + (PX_NUM * 10) // base status modbus register



#define NUM_SERVOS 5
#define SERVO_MIN_POS 2297 //in microseconds
#define SERVO_MAX_POS 647 // in microseconds
#define SERVO_SPEED (SERVO_MIN_POS - SERVO_MAX_POS) / 20

#define NUM_LEDS 5

#define EXT_CONTROL 0
#define DEMO_MODE 1

uint8_t sysState = EXT_CONTROL;

enum {
  PX_ERR = -1,
  PX_OK
};

void attachAllServos(void);
void detachAllServos(void);


//PXServo keeps track of servo state
struct PXServo{
  Servo pxServo;
  int16_t pxPos;
  int16_t pxGoalPos;
};

//PXLed keeps track ofUV led states
struct PXLed{
  uint8_t pin; //pinnumber
  uint8_t state; //0 off, 1 on
};

PXServo ServoCtl[NUM_SERVOS];
PXLed Leds[NUM_LEDS];


char ssid[] = SSID  ;        // your network SSID (name)
char pass[] = PW;                    // your network password
WifiControl pxWifi(ssid, pass, PX_NUM);

ModbusIP pxModbus;


int8_t demoState = 0;
void demoCallback(uint32_t dTime, px_mode_t mode);
ModeControl pxMC(3, &demoCallback, 5000, &pxWifi);



//keeping track ofpx3State
int8_t px3State[2] = {0, 0};

void setLed(int8_t idx, int8_t state);

uint32_t mytime = 0;
uint32_t wifiReconTime = 0;



void setup() {

  /*
  * Initialize servos and leds
  */

  ServoCtl[0].pxServo.attach(16, SERVO_MIN, SERVO_MAX); // D0
  ServoCtl[1].pxServo.attach(5, SERVO_MIN, SERVO_MAX); // D1
  ServoCtl[2].pxServo.attach(4, SERVO_MIN, SERVO_MAX); // D2
  ServoCtl[3].pxServo.attach(0, SERVO_MIN, SERVO_MAX); // D3
  ServoCtl[4].pxServo.attach(2, SERVO_MIN, SERVO_MAX); //D4


  Leds[0].pin = 14;
  Leds[1].pin = 10;
  Leds[2].pin = 12;
  Leds[3].pin = 13;
  Leds[4].pin = 15;


  for(uint8_t i = 0; i < NUM_LEDS; i++){
    Leds[i].state = 0;
    pinMode(Leds[i].pin, OUTPUT);
    digitalWrite(Leds[i].pin, Leds[i].state);
  }

  for(uint8_t i = 0; i < NUM_SERVOS; i++ ){
    ServoCtl[i].pxServo.writeMicroseconds(SERVO_MAX_POS);
    ServoCtl[i].pxGoalPos = SERVO_MAX_POS;
    ServoCtl[i].pxPos = SERVO_MAX_POS;
  }

  //Serial for debugging
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);

  delay(5000);

  

  pxWifi.setPreConn(detachAllServos);
  pxWifi.setPostConn(attachAllServos);

  //time out used for waiting for a connection to occur
  //useful to change during testing to speed things up when dev-ing with no C&C
  pxWifi.setTimeOut(30000);
  
  //attempt to connect to C&C when not forced into demo mode
  if(digitalRead(3) == HIGH){
    Serial.println("Connecting to C&C...");
    int8_t res = pxWifi.init();
    if(res == -1){
      Serial.println("No C&C found, starting up in demo mode!");
    }
  }

  pxModbus.server(502);
  pxModbus.addHreg(PX_REG, 0);
  pxModbus.addIreg(PX_REG, 0);
  pxModbus.addHreg(PX_LED_REG, 0);
  pxModbus.addIreg(PX_LED_REG, 0);
  pxModbus.addHreg(PX_STATE_REG, PX_OK);

  pxMC.init();//initialize modeControl

  Serial.println("Setup complete");
}


/*
* after receiving a modbus message with a single int this function is called
* it sets the state for 0 to 5, it affects only the the open/closed state of PX-III.
* 0 being all closed, 5 being all open
*
* the states are translated to action by setting the goal positions of the servo motors
* actual motion is executed in the main loop
*/

void setState(int8_t state){

  px3State[0] = state;

  for(int8_t i = 0; i < NUM_SERVOS; i++){
    if(i < px3State[0]){
      ServoCtl[i].pxGoalPos = SERVO_MIN_POS;

    }else{
     ServoCtl[i].pxGoalPos = SERVO_MAX_POS;

    }
  }



}

void setState(void){


  for(int8_t i = 0; i < NUM_SERVOS; i++){
    if(i < px3State[0]){
      ServoCtl[i].pxGoalPos = SERVO_MIN_POS;

    }else{
     ServoCtl[i].pxGoalPos = SERVO_MAX_POS;

    }
  }



}


/*
* turns on / off leds according to int received via modbus
*/


void setLeds(int8_t state){
  //int8_t ledsOn = (int8_t)msg.getInt(0);

  px3State[1] = state;
  int8_t ledsOn = px3State[1];
  for(int8_t i = 0; i < NUM_LEDS; i++){
    if(i < ledsOn){
      setLed(i, 1);
    }else{
      setLed(i, 0);
    }
  }
}

void setLeds(void){

  for(int8_t i = 0; i < NUM_LEDS; i++){
    if(i < px3State[1]){
      setLed(i, 1);
    }else{
      setLed(i, 0);
    }
  }

}



void loop() {


  pxModbus.task();
  pxMC.run();

  //this copies the holding reg value to ireg
  if(pxModbus.Hreg(PX_REG) != pxModbus.Ireg(PX_REG)){
    pxModbus.Ireg(PX_REG, pxModbus.Hreg(PX_REG));
  }

  if(pxModbus.Hreg(PX_LED_REG) != pxModbus.Ireg(PX_LED_REG)){
    pxModbus.Ireg(PX_LED_REG, pxModbus.Hreg(PX_LED_REG));
  }

  if(px3State[0] != pxModbus.Ireg(PX_REG)){
    setState((int8_t)pxModbus.Ireg(PX_REG));
  }

  if(px3State[1] != pxModbus.Ireg(PX_LED_REG)){
    setLeds((int8_t)pxModbus.Ireg(PX_LED_REG));
  }

  if(pxWifi.getStatus() != WL_CONNECTED && px3State[0] == 0 && px3State[1] == 0){
    int currPos = ServoCtl[0].pxServo.readMicroseconds();
    if(currPos == ServoCtl[0].pxGoalPos){
      pxWifi.reConn();
    }
  }
  //update the servos
  if(millis() - mytime > 5){
    for(int8_t i = 0; i < NUM_SERVOS; i++){
      int currPos = ServoCtl[i].pxServo.readMicroseconds();
      
      ServoCtl[i].pxServo.write((ServoCtl[i].pxGoalPos * 0.05) + (currPos * 0.95));

    }

    mytime = millis();
  }

  


}


//set the led to on/off and update it's state
void setLed(int8_t idx, int8_t state){
  Leds[idx].state = state;
  digitalWrite(Leds[idx].pin, Leds[idx].state);
}


void demoCallback(uint32_t dTime, px_mode_t mode)
{
  
  if(mode == PX_DEMO_MODE){
    if(pxModbus.Ireg(PX_LED_REG) == 5){
      pxModbus.Hreg(PX_LED_REG, 0);
      
    }
    int16_t stateReg = pxModbus.Ireg(PX_REG);
    stateReg++;
    pxModbus.Hreg(PX_REG, stateReg);
  

    if(stateReg == 5){
      int16_t ledReg = pxModbus.Ireg(PX_LED_REG);
      ledReg++;
      pxModbus.Hreg(PX_LED_REG, ledReg);
      
      
    }
    if(stateReg > 5){
      pxModbus.Hreg(PX_REG, 0);

    }
  }else if(mode == PX_CC_MODE){
    pxModbus.Hreg(PX_REG, 0);
    pxModbus.Hreg(PX_LED_REG, 0);
  }

    
}

void attachAllServos()
{
  ServoCtl[0].pxServo.attach(16, SERVO_MIN, SERVO_MAX); // D0
  ServoCtl[1].pxServo.attach(5, SERVO_MIN, SERVO_MAX); // D1
  ServoCtl[2].pxServo.attach(4, SERVO_MIN, SERVO_MAX); // D2
  ServoCtl[3].pxServo.attach(0, SERVO_MIN, SERVO_MAX); // D3
  
  
  ServoCtl[4].pxServo.attach(2, SERVO_MIN, SERVO_MAX); //D4
}

void detachAllServos()
{
  ServoCtl[0].pxServo.detach(); // D0
  ServoCtl[1].pxServo.detach(); // D1
  ServoCtl[2].pxServo.detach(); // D2
  ServoCtl[3].pxServo.detach(); // D3
  ServoCtl[4].pxServo.detach(); //D4
}
