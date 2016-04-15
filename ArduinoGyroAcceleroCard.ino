
/* Author = helscream (Omer Ikram ul Haq)
Last edit date = 2014-10-08
Website: http://hobbylogs.me.pn/?p=47
Location: Pakistan
Ver: 0.1 beta --- Start
Ver: 0.2 beta --- Bug fixed for calculating "angle_y_accel" and "angle_x_accel" in "Gyro_header.ino" file
*/

#include <SPI.h>
#include <Wire.h>
#include "gyro_accel.h"
#include "Emotion.h"
#include <Canutil.h>
#include <MCP2510.h>
// Defining constants
#define dt 20                       // time difference in milli seconds
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.95             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)
#define BUFFER_SIZE 32



#define LED 20
#define CHECK_CAN 1000
#define LECTURE_MPU6050 0
#define TRAITEMENT_DONNEE 1
#define ENVOI_INFO 10
#define AFFICHAGE_DONNEE 2
#define ALERTE_CHUTE 5
#define ALERTE_SECOUSSE 3
#define ALERTE_RETOURNER 4
#define ALERTE_TEMPERATURE_INTERNE 6
#define ALERTE_TEMPERATURE_EXTERNE 7
#define ALERTE_HUMIDITY_INTERNE 8
#define ALERTE_HUMIDITY_EXTERNE 9



#define OFFSET_RASP 0x20000
#define OWN_NODE_ID 0x00002
//id LSB = priorité, si 0 prioritaire,  15 bits suivant id hardware (1), et apres 8bits id messages
#define IdAlertesecousse  0x1     // 0, 1, 1
#define IdAlerteretourner 0x2      //0, 1, 2
#define IdAlerteChute 0x3         //0, 1, 3
//#define IdAlerteChoc 0x4       //0, 1, 4
#define IdAlerteTemperatureInterne 0x5  //0, 1, 5
#define IdAlerteTemperatureExterne 0x6  //0, 1, 6
#define IdAlerteHumidityInterne 0x7 //0, 1, 7
#define IdAlerteHumidityExterne 0x8  //0, 1, 8


#define IDangleFromNorth 0x9  
//#define IDangleFromCheckPoint 0xA      
#define IDvitesseXYZ 0xB      
#define IDpositionXYZ 0xC     
#define IDangle 0xD     
#define IDTemperature 0xE   

#define addr 0x1E //I2C Address for The HMC5883

#define node_id 0x003
#define shock_alert_id 0x20001
#define turned_alert_id 0x20002
#define fall_alert_id 0X20003


// *********************************************************************
//    Global Variables
// *********************************************************************
unsigned long t = 0, t2, tState; // Time Variables
float angle_x_gyro = 0, angle_y_gyro = 0, angle_z_gyro = 0, angle_x_accel = 0, angle_y_accel = 0, angle_z_accel = 0, angle_x = 0, angle_y = 0, angle_z = 0;
float vitesse_x = 0, vitesse_y = 0, vitesse_z = 0, position_x = 0, position_y = 0, position_z = 0;
float accelX[3];
float accelY[3];
float accelZ[3];




///////////////CAN shit
MCP2510  can_dev (9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);
volatile uint16_t msgID;
volatile int recSize;
volatile uint8_t canDataReceived[8];
uint8_t txstatus;


/////magneto
  int x, y, z; //triple axis data
  float angleNorth;


////
//Valeur transfert
int vitesseX, vitesseY, vitesseZ, acceleX, acceleY, acceleZ, positionX, positionY, positionZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ, tempInte, tempExte;

//Temp de Yong
unsigned long currentMillis = millis();
int analogPin = 5;  
int val = 0;        
const int bits=10;         
float volts=0.0;         
float tempExt=0.0;
float dT=0.0;           
const float Vmin=0.0;
const float Vmax=5.0;
long previousMillis = 0;
float fs=100.0; 
//int interval = 10;
int interval=1000/fs;


//////////////Machne etat
int machine =0;

//LED
int dataIn = 14; //DIN //avant : 36
int load = 15;   //CS  //avant : 37
int clock = 16;  //CLK //avant : 38
int maxInUse = 4;    //change this variable to set how many MAX7219's you'll use
int e = 0;    
//Variable globales : 

//Tableaux d'entiers contenant les indices des pattern que l'on va utiliser 

//Pour l'oeil gauche

int indexOfHappyEye1[]={0,1,2,1,0,3,4,3,0};
int indexOfInLoveEye1[]={0,9,10,10,11,11,10,10,11,11,10,10,11,11,10,10,11,11,10,10,10,9,0};
int indexOfEyeClosesEye1[]={0,5,6,7,8,8,8,8,8,8,8,8,8,8,8,8,7,6,5,0};
int indexOfHungryEye1[]={0,13,14,15,12,15,16,17,18,17,16,15,12,14,13};
int indexOfCrazyEye1[]={0,3,4,19,20,21,22,23,24,25,26,27,28,29,4,3,0};
int indexOfSadEye1[]={0,5,6,34,31,32,30,32,33,34,35,34,35,6,5,0};
int indexOfNeutre1Eye1[]={0,36,0};
int indexOfNeutre2Eye1[]={0,0,0};
int indexOfNeutre3Eye1[]={0,1,0};
int indexOfNeutre4Eye1[]={0,0,0};

//Pour l'oeil droit

//int indexOfHappyEye2[]={0,1,2,1,0,3,4,3,0};
//int indexOfInLoveEye2[]={0,9,10,10,11,11,10,10,11,11,10,10,11,11,10,10,11,11,10,10,10,9,0};
//int indexOfEyeClosesEye2[]={0,5,6,7,8,8,8,8,8,8,8,8,8,8,8,8,7,6,5,0};
//int indexOfHungryEye2[]={0,13,14,15,12,15,16,17,18,17,16,15,12,14,13};
int indexOfCrazyEye2[]={0,3,4,29,28,27,26,25,24,23,22,21,20,19,4,3,0};
int indexOfSadEye2[]={0,5,6,34,37,38,39,38,40,34,35,34,35,6,5,0};
//int indexOfNeutre1Eye2[]={0,36,0};
//int indexOfNeutre2Eye2[]={0,0,0};
//int indexOfNeutre3Eye2[]={0,1,0};
//int indexOfNeutre4Eye2[]={0,0,0};

//Pour la bouche gauche 
int indexOfHappyMouth1[]={0,1,2,3,3,3,2,1,0};
int indexOfInLoveMouth1[]={0,4,5,6,12,13,14,15,10,16,17,18,19,20,10,15,14,13,12,6,5,4,0};
int indexOfEyeClosesMouth1[]={0,4,5,6,12,13,14,15,10,11,10,10,15,14,13,12,6,5,4,0};
int indexOfHungryMouth1[]={0,4,5,6,7,7,7,7,7,7,7,6,5,4,0};
int indexOfCrazyMouth1[]={0,8,9,9,9,9,9,9,9,9,9,9,9,8,0};
int indexOfSadMouth1[]={0,4,5,6,7,22,21,21,21,21,22,7,6,5,4,0};
int indexOfNeutre1Mouth1[]={0,0,0};
int indexOfNeutre2Mouth1[]={0,1,0};
int indexOfNeutre3Mouth1[]={0,0,0};
int indexOfNeutre4Mouth1[]={0,4,0};

//Pour la bouche droite 
int indexOfHappyMouth2[]={0,1,2,3,3,3,2,1,0};
int indexOfInLoveMouth2[]={0,4,5,6,12,13,14,15,10,16,17,18,19,20,10,15,14,13,12,6,5,4,0};
int indexOfEyeClosesMouth2[]={0,4,5,6,12,13,14,15,10,11,10,10,15,14,13,12,6,5,4,0};
int indexOfHungryMouth2[]={0,4,5,6,7,7,7,7,7,7,7,6,5,4,0};
int indexOfCrazyMouth2[]={0,8,9,9,9,9,9,9,9,9,9,9,9,8,0};
int indexOfSadMouth2[]={0,4,5,6,7,22,21,21,21,21,22,7,6,5,4,0};
int indexOfNeutre1Mouth2[]={0,0,0};
int indexOfNeutre2Mouth2[]={0,1,0};
int indexOfNeutre3Mouth2[]={0,0,0};
int indexOfNeutre4Mouth2[]={0,4,0};

//Compteur qui va être incrémenté à chaque fois qu'on va rentrer dans mon état 
int compteur;

// Indice du tableau du pattern joué en ce moment 
int indexOfEmotion;

//Emotion qui est en train de joué 
Emotion emotionPlayed;


                     // define max7219 registers
byte max7219_reg_noop        = 0x00;
byte max7219_reg_digit0      = 0x01;
byte max7219_reg_digit1      = 0x02;
byte max7219_reg_digit2      = 0x03;
byte max7219_reg_digit3      = 0x04;
byte max7219_reg_digit4      = 0x05;
byte max7219_reg_digit5      = 0x06;
byte max7219_reg_digit6      = 0x07;
byte max7219_reg_digit7      = 0x08;
byte max7219_reg_decodeMode  = 0x09;
byte max7219_reg_intensity   = 0x0a;
byte max7219_reg_scanLimit   = 0x0b;
byte max7219_reg_shutdown    = 0x0c;
byte max7219_reg_displayTest = 0x0f;

// *********************************************************************
// Main Code
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();

  MPU6050_ResetWake();
  MPU6050_SetGains(0, 1); // Setting the lows scale
  MPU6050_SetDLPF(0); // Setting the DLPF to inf Bandwidth for calibration
  MPU6050_OffsetCal();
  MPU6050_SetDLPF(6); // Setting the DLPF to lowest Bandwidth


//////////////////////////////////////////////////////////////////////////////////////////////////////:
//setup led

 //Instancification de la structure emotion avec differentes emotions 

  emotion[0].name="happy";
  emotion[0].pattern_number=sizeof(indexOfHappyEye1)/2;
  emotion[0].pattern_delay=150; //150
  initialization(indexOfHappyEye1,emotion[0].pattern_number,emotion[0].emotion_array_eye_1);
  //initialization(indexOfHappyEye1,emotion[0].pattern_number,emotion[0].emotion_array_eye_2);
  initialization(indexOfHappyMouth1,emotion[0].pattern_number,emotion[0].emotion_array_mouth_1);  
  initialization(indexOfHappyMouth2,emotion[0].pattern_number,emotion[0].emotion_array_mouth_2);

  emotion[1].name="inLove";
  emotion[1].pattern_number=sizeof(indexOfInLoveEye1)/2;
  emotion[1].pattern_delay=1000; //150
  initialization(indexOfInLoveEye1,emotion[1].pattern_number,emotion[1].emotion_array_eye_1);
  //initialization(indexOfInLoveEye1,emotion[1].pattern_number,emotion[1].emotion_array_eye_2);
  initialization(indexOfInLoveMouth1,emotion[1].pattern_number,emotion[1].emotion_array_mouth_1);
  initialization(indexOfInLoveMouth2,emotion[1].pattern_number,emotion[1].emotion_array_mouth_2);

  emotion[2].name="eyeCloses";
  emotion[2].pattern_number=sizeof(indexOfEyeClosesEye1)/2;
  emotion[2].pattern_delay=1000;
  initialization(indexOfEyeClosesEye1,emotion[2].pattern_number,emotion[2].emotion_array_eye_1);
  //initialization(indexOfEyeClosesEye1,emotion[2].pattern_number,emotion[2].emotion_array_eye_2);
  initialization(indexOfEyeClosesMouth1,emotion[2].pattern_number,emotion[2].emotion_array_mouth_1);
  initialization(indexOfEyeClosesMouth2,emotion[2].pattern_number,emotion[2].emotion_array_mouth_2);

  emotion[3].name="hungry";
  emotion[3].pattern_number=sizeof(indexOfHungryEye1)/2;
  emotion[3].pattern_delay=1000; //175
  initialization(indexOfHungryEye1,emotion[3].pattern_number,emotion[3].emotion_array_eye_1);
  //initialization(indexOfHungryEye1,emotion[3].pattern_number,emotion[3].emotion_array_eye_2);
  initialization(indexOfHungryMouth1,emotion[3].pattern_number,emotion[3].emotion_array_mouth_1);
  initialization(indexOfHungryMouth2,emotion[3].pattern_number,emotion[3].emotion_array_mouth_2);

  emotion[4].name="crazy";
  emotion[4].pattern_number=sizeof(indexOfCrazyEye1)/2;
  emotion[4].pattern_delay=1000; //50
  initialization(indexOfCrazyEye1,emotion[4].pattern_number,emotion[4].emotion_array_eye_1);
  //initialization(indexOfCrazyEye2,emotion[4].pattern_number,emotion[4].emotion_array_eye_2);
  initialization(indexOfCrazyMouth1,emotion[4].pattern_number,emotion[4].emotion_array_mouth_1);
  initialization(indexOfCrazyMouth2,emotion[4].pattern_number,emotion[4].emotion_array_mouth_2);

  emotion[5].name="sad";
  emotion[5].pattern_number=sizeof(indexOfSadEye1)/2;
  emotion[5].pattern_delay=1000; //350
  initialization(indexOfSadEye1,emotion[5].pattern_number,emotion[5].emotion_array_eye_1);
  //initialization(indexOfSadEye2,emotion[5].pattern_number,emotion[5].emotion_array_eye_2);
  initialization(indexOfSadMouth1,emotion[5].pattern_number,emotion[5].emotion_array_mouth_1);
  initialization(indexOfSadMouth2,emotion[5].pattern_number,emotion[5].emotion_array_mouth_2);

  emotion[10].name="neutre1";
  emotion[10].pattern_number=sizeof(indexOfNeutre1Eye1)/2;
  emotion[10].pattern_delay=1000; //400
  initialization(indexOfNeutre1Eye1,emotion[10].pattern_number,emotion[10].emotion_array_eye_1);
  //initialization(indexOfNeutre1Eye1,emotion[10].pattern_number,emotion[10].emotion_array_eye_2);
  initialization(indexOfNeutre1Mouth1,emotion[10].pattern_number,emotion[10].emotion_array_mouth_1);  
  initialization(indexOfNeutre1Mouth2,emotion[10].pattern_number,emotion[10].emotion_array_mouth_2);

  emotion[11].name="neutre2";
  emotion[11].pattern_number=sizeof(indexOfNeutre2Eye1)/2;
  emotion[11].pattern_delay=1000; //400
  initialization(indexOfNeutre2Eye1,emotion[11].pattern_number,emotion[11].emotion_array_eye_1);
  //initialization(indexOfNeutre2Eye1,emotion[11].pattern_number,emotion[11].emotion_array_eye_2);
  initialization(indexOfNeutre2Mouth1,emotion[11].pattern_number,emotion[11].emotion_array_mouth_1);  
  initialization(indexOfNeutre2Mouth2,emotion[11].pattern_number,emotion[11].emotion_array_mouth_2);

  emotion[12].name="neutre3";
  emotion[12].pattern_number=sizeof(indexOfNeutre3Eye1)/2;
  emotion[12].pattern_delay=1000; //400
  initialization(indexOfNeutre3Eye1,emotion[12].pattern_number,emotion[12].emotion_array_eye_1);
  //initialization(indexOfNeutre3Eye1,emotion[12].pattern_number,emotion[12].emotion_array_eye_2);
  initialization(indexOfNeutre3Mouth1,emotion[12].pattern_number,emotion[12].emotion_array_mouth_1);  
  initialization(indexOfNeutre3Mouth2,emotion[12].pattern_number,emotion[12].emotion_array_mouth_2);

  emotion[13].name="neutre4";
  emotion[13].pattern_number=sizeof(indexOfNeutre4Eye1)/2;
  emotion[13].pattern_delay=1000; //400
  initialization(indexOfNeutre4Eye1,emotion[13].pattern_number,emotion[13].emotion_array_eye_1);
  //initialization(indexOfNeutre4Eye1,emotion[13].pattern_number,emotion[13].emotion_array_eye_2);
  initialization(indexOfNeutre4Mouth1,emotion[13].pattern_number,emotion[13].emotion_array_mouth_1);  
  initialization(indexOfNeutre4Mouth2,emotion[13].pattern_number,emotion[13].emotion_array_mouth_2);

  indexOfEmotion=0;

  compteur=-1;
  
  pinMode(dataIn, OUTPUT);
  pinMode(clock,  OUTPUT);
  pinMode(load,   OUTPUT);

  //beginSerial(9600);
  digitalWrite(13, HIGH); 

  //initiation of the max 7219
  maxAll(max7219_reg_scanLimit, 0x07);      
  maxAll(max7219_reg_decodeMode, 0x00);  // using a led matrix (not digits)
  maxAll(max7219_reg_shutdown, 0x01);    // not in shutdown mode
  maxAll(max7219_reg_displayTest, 0x00); // no display test
   for (e=1; e<=8; e++) {    // empty registers, turn all LEDs off 
    maxAll(e,0);
  }
  maxAll(max7219_reg_intensity, 0x0f & 0x0f);    // the first 0x0f is the value you can set
                                                  // range: 0x00 to 0x0f  

////////////////////////////////////////////////////////////////////////////////////////////////////////
//setup can

  can_dev.write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

  canutil.setClkoutMode(0, 0); // disables CLKOUT
  canutil.setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input

  canutil.setOpMode(4); // sets configuration mode
  // IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
  // filters and acceptance masks can be modified

 //PCB A FAIRE canutil.waitOpMode(4);  // waits configuration mode
 Serial.println("can you hear me 4");
  can_dev.write(CNF1, 0x03); // SJW = 1, BRP = 3
  can_dev.write(CNF2, 0b10110001); //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
  can_dev.write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5

  // SETUP MASKS / FILTERS FOR CAN
  canutil.setRxOperatingMode(1, 1, 0);  // standard ID messages only  and rollover
  canutil.setAcceptanceFilter(0x102, 0, 0, 1); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil.setAcceptanceFilter(0x101, 0, 0, 5); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 0
  canutil.setAcceptanceFilter(0x100, 0, 0, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, 1 = extended, filter# 1
  canutil.setAcceptanceMask(0x000, 0x00000000, 0); // 0 <= stdID <= 2047, 0 <= extID <= 262143, buffer# 0

  canutil.setOpMode(0); // sets normal mode

  t = millis();
}
void loop() {
  t = millis();

  switch (machine) {

    case LECTURE_MPU6050 :
      MPU6050_ReadData();

      //Tell the HMC what regist to begin writing data into
      Wire.beginTransmission(addr);
      Wire.write(0x03); //start with register 3.
      Wire.endTransmission();

      Wire.requestFrom(addr, 6);
      if (6 <= Wire.available()) {
        x = Wire.read() << 8; //MSB  x
        x |= Wire.read(); //LSB  x
        z = Wire.read() << 8; //MSB  z
        z |= Wire.read(); //LSB z
        y = Wire.read() << 8; //MSB y
        y |= Wire.read(); //LSB y
      }

      machine = 1;
      break;

    case TRAITEMENT_DONNEE :


    
      angle_x_gyro = (gyro_x_scalled * ((float)dt / 1000) + angle_x);
      angle_y_gyro = (gyro_y_scalled * ((float)dt / 1000) + angle_y);
      angle_z_gyro = (gyro_z_scalled * ((float)dt / 1000) + angle_z);

      angle_z_accel = atan(accel_z_scalled / (sqrt(accel_y_scalled * accel_y_scalled + accel_x_scalled * accel_x_scalled))) * (float)rad2degree;
      angle_y_accel = -atan(accel_y_scalled / (sqrt(accel_y_scalled * accel_y_scalled + accel_z_scalled * accel_z_scalled))) * (float)rad2degree;
      angle_x_accel = atan(accel_x_scalled / (sqrt(accel_x_scalled * accel_x_scalled + accel_z_scalled * accel_z_scalled))) * (float)rad2degree;

      angle_x = Filter_gain * angle_x_gyro + (1 - Filter_gain) * angle_x_accel;
      angle_y = Filter_gain * angle_y_gyro + (1 - Filter_gain) * angle_y_accel;
      angle_z = Filter_gain * angle_z_gyro + (1 - Filter_gain) * angle_z_accel;



///////Mise en tableau des valeurs d'accélération
accelX[0] = accelX[1];
accelX[1] = accelX[2];
accelX[2] = accel_x_scalled;
//////////////////////////////////////////////
accelY[0] = accelY[1];
accelY[1] = accelY[2];
accelY[2] = accel_y_scalled;
//////////////////////////////////////////////
accelZ[0] = accelZ[1];
accelZ[1] = accelZ[2];
accelZ[2] = accel_z_scalled;

      

      //////////Tester véracité de la vitesse et de la position

      vitesse_x = (accel_x_scalled * ((float)dt / 1000) + vitesse_x);
      vitesse_y = (accel_y_scalled * ((float)dt / 1000) + vitesse_y);
      vitesse_z = (accel_z_scalled * ((float)dt / 1000) + vitesse_z);

      position_x = (vitesse_x * ((float)dt / 1000) + position_x);
      position_y = (vitesse_y * ((float)dt / 1000) + position_y);
      position_z = (vitesse_z * ((float)dt / 1000) + position_z);

      angleNorth = (round(atan2(x, y) * 180 / 3.14159265) - 90) * 2;

      /////Temp Yong

  if(currentMillis - previousMillis > interval) 
     {
        previousMillis = currentMillis; 
        val = analogRead(analogPin);   
        volts = Vmin +  (Vmax-Vmin)*val/(pow(2,bits)-1);
        tempExt = (90/35)*((5/volts*137)-237);
        //dT = dT + (float)interval / 1000.0;
        dT=dT + 1.0/fs;
        Serial.print("Time= ");       
        Serial.print(dT,4);            
        Serial.print(" ");             
        Serial.print("temp= ");       
        Serial.print(tempExt,4);       
        Serial.println("");         
        delay(100);     
     }
/////////////////


/////Changement des données pour transmission
vitesseX = (int8_t)round(vitesse_x);
vitesseY = (int8_t)round(vitesse_y);
vitesseZ = (int8_t)round(vitesse_z);
acceleX = (int)round(accel_x_scalled);
acceleY = (int)round(accel_y_scalled);
acceleZ = (int)round(accel_z_scalled);
positionX = (int8_t)round(position_x);
positionY = (int8_t)round(position_y);
positionZ = (int8_t)round(position_z);
gyroX = (int)round(angle_x_gyro);
gyroY = (int)round(angle_y_gyro);
gyroZ = (int)round(angle_z_gyro);
angleX = (int)round(angle_x);
angleY = (int)round(angle_y);
angleZ = (int)round(angle_z);
tempInte = (int8_t)round(temp_scalled);
tempExte = (int8_t)round(tempExt);
//////////////////////////

      if (machine==3) {
        machine = 3;
      } else if (accel_z_scalled < 0){
        machine = 4;
      } else if (vitesse_x > 60 || vitesse_y > 60 || vitesse_z > 60) {
        machine = 5;
      } else if (temp_scalled > 50) {
        machine = 6;
      } else if (tempExt > 40 || tempExt < 0) {
        machine = 7;
      } else if (machine==8) {
        machine = 8;
      } else if (machine==9) {
        machine = 9;
      } else {
        machine = 2;
      }
      break;



    case AFFICHAGE_DONNEE :
      Serial.println(" ");
      Serial.println("---------------------------------------------------------------------------------------------------------");
      Serial.println(" ");

      Serial.print("gyro_x_scalled: ");
      Serial.println(gyro_x_scalled);
      Serial.print("gyro_y_scalled: ");
      Serial.println(gyro_y_scalled);
      Serial.print("gyro_z_scalled: ");
      Serial.println(gyro_z_scalled);

      Serial.println(" ");

      Serial.print("accel_x_scalled: ");
      Serial.println(accel_x_scalled);
      Serial.print("accel_y_scalled: ");
      Serial.println(accel_y_scalled);
      Serial.print("accel_z_scalled: ");
      Serial.println(accel_z_scalled);

      Serial.println(" ");

      Serial.print("angle_x_gyro: ");
      Serial.println(angle_x_gyro);
      Serial.print("angle_y_gyro: ");
      Serial.println(angle_y_gyro);
      Serial.print("angle_z_gyro: ");
      Serial.println(angle_z_gyro);

      Serial.println(" ");

      Serial.print("angle_x_accel: ");
      Serial.println(angle_x_accel);
      Serial.print("angle_y_accel: ");
      Serial.println(angle_y_accel);
      Serial.print("angle_z_accel: ");
      Serial.println(angle_z_accel);

      Serial.println(" ");

      Serial.print("vitesse_x: ");
      Serial.println(vitesse_x);
      Serial.print("vitesse_y: ");
      Serial.println(vitesse_y);
      Serial.print("vitesse_z: ");
      Serial.println(vitesse_z);

      Serial.println(" ");

      Serial.print("position_x: ");
      Serial.println(position_x);
      Serial.print("position_y: ");
      Serial.println(position_y);
      Serial.print("position_z: ");
      Serial.println(position_z);

      Serial.println(" ");

      Serial.print("angle_x: ");
      Serial.println(angle_x);
      Serial.print("angle_y: ");
      Serial.println(angle_y);
      Serial.print("angle_z: ");
      Serial.println(angle_z);


            Serial.println(" ");

      Serial.print("temperature: ");
      Serial.println(temp_scalled);


      Serial.println(" ");

      Serial.print("Load ");
      Serial.println(((float)(millis() - t) / (float)dt) * 100);

      Serial.print("X Value Magneto: ");
      Serial.println(x);
      Serial.print("Y Value Magneto: ");
      Serial.println(y);
      Serial.print("Z Value Magneto: ");
      Serial.println(z);
      Serial.print("angle: ");
      Serial.println(angleNorth);
      Serial.println();
      machine = 10;
      break;


    case ALERTE_SECOUSSE :
      //TODO
      //Envoi du message d'Alerte: secousse
      //donnée: d'accélération, 2 float (accel_x_scalled ou accel_y_scalled ou accel_z_scalled     

      // IdAlertesecousse  0b000000000000000100000001
      canutil.setTxBufferID(node_id, shock_alert_id, 1, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
      //      canutil.setTxBufferDataField(message, 0);   // fills TX buffer
      canutil.setTxBufferDataLength(0, 0, 0);
      canutil.messageTransmitRequest(0, 1, 3);
      do {
        txstatus = 0;
        txstatus = canutil.isTxError(0);  // checks tx error
        txstatus = txstatus + canutil.isArbitrationLoss(0);  // checks for arbitration loss
        txstatus = txstatus + canutil.isMessageAborted(0);  // ckecks for message abort
        txstatus = txstatus + canutil.isMessagePending(0);   // checks transmission
      }
      while (txstatus != 0);
      machine = 1000;
      break;


      
    case ALERTE_RETOURNER :
      //TODO
      //Envoi du message d'Alerte: robot retourné
      //donnée: angle, 2 float (angle_x et angle_y)

      // #define IdAlerteretourner 0b000000000000000100000010
      canutil.setTxBufferID(node_id, turned_alert_id, 1, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
      //      canutil.setTxBufferDataField(message, 0);   // fills TX buffer
      canutil.setTxBufferDataLength(0, 0, 0);
      canutil.messageTransmitRequest(0, 1, 3);
      do {
        Serial.println("ALEEEEEEEEEEEEEEEERTE");
        txstatus = 0;
        txstatus = canutil.isTxError(0);  // checks tx error
        txstatus = txstatus + canutil.isArbitrationLoss(0);  // checks for arbitration loss
        txstatus = txstatus + canutil.isMessageAborted(0);  // ckecks for message abort
        txstatus = txstatus + canutil.isMessagePending(0);   // checks transmission
      }
      while (txstatus != 0);

      machine = 1000;
      break;


    case ALERTE_CHUTE :
      //TODO
      //Envoi du message d'Alerte: Chute
      //donnée accélération, 2 float  (accel_x_scalled ou accel_y_scalled ou accel_z_scalled)
      canutil.setTxBufferID(node_id, fall_alert_id, 1, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
      //      canutil.setTxBufferDataField(message, 0);   // fills TX buffer
      canutil.setTxBufferDataLength(0, 0, 0);
      canutil.messageTransmitRequest(0, 1, 3);
      do {
        txstatus = 0;
        txstatus = canutil.isTxError(0);  // checks tx error
        txstatus = txstatus + canutil.isArbitrationLoss(0);  // checks for arbitration loss
        txstatus = txstatus + canutil.isMessageAborted(0);  // ckecks for message abort
        txstatus = txstatus + canutil.isMessagePending(0);   // checks transmission
      }
      while (txstatus != 0);

      //#define IdAlerteChute 0b000000000000000100000011

      machine = 1000;
      break;


case ALERTE_TEMPERATURE_INTERNE:



      machine = 1000;
break;


case ALERTE_TEMPERATURE_EXTERNE:




      machine = 1000;
break;


case ALERTE_HUMIDITY_INTERNE:




      machine = 1000;
break;


case ALERTE_HUMIDITY_EXTERNE:



      machine = 1000;
break;



    case ENVOI_INFO :
      //TODO
      /*
      envoi les donnée d'angle et de vitesse dans le CAN
      vitesse = 3 float   (vitesse_x, vitesse_y, vitesse_z)
      position = 3 float  (position_x, position_y, position_z)
      angle = 3 float   (angle_x, angle_y, angle_z)
      angle par rapport au nord = float  (angleNorth)
      IDangleFromNorth 0b100000000000000100000101
       IDvitesseXYZ 0b100000000000000100000111
       IDpositionXYZ 0b100000000000000100001000
       IDangle 0b100000000000000100001001
      */
      machine = 1000;
      break;


case LED :
t2 = millis();

emotionPlayed=emotion[indexOfEmotion];

  //entier qui permettra de vérifier si on est à la fin de l'émotion 
  int testEndEmotion;

  //incrémentation de ma variable compteur 
  compteur=compteur+1;
  
  //On affiche le pattern
  testEndEmotion=printPattern(emotionPlayed, compteur, 1);

  //test pour savoir si on est à la fin de l'émotion, dans ces cas la, indexOfEmotion prendra une valeur random dans le tableau l'Emotion
   if (testEndEmotion==-1) {
    if (indexOfEmotion<10) {
      indexOfEmotion=random(10,14);
      Serial.println(indexOfEmotion);
    }
    else {
      indexOfEmotion=random(0,6);
    }
    compteur=-1;
    //indexOfEmotion=indexOfEmotion+1;
    //Serial.println(indexOfEmotion);
    testEndEmotion=0;
   }
   machine = 0;
break;


      

   case CHECK_CAN :


      //TODO

      //état de lecture du CAN pour recevoir les information de la raspberry
      //si il y a demande d'info, machine =0;

if ((millis()- t2) > emotionPlayed.pattern_delay) {   ////////////////////////////////////////////////////////////////////DEMANDER A HELENE
  machine = 20;
} else machine = 0;
      break;
  }



  while ((millis() - t) < dt) { // Making sure the cycle time is equal to dt
    // Do nothing
  }


  //delay(200);
}










//************************************************* Fonction à la con en lien avec les leds parce que la création du cpp est galère ***************************************//

//Procédure qui permettera d'initialier les tableaux contenant les indices des patterns
//Paramètres : 
//indexOfEmotion : tableau contenant les indices 
//sizeOfArray : taille du tableau 
//array : le tableau qui sera ensuite associé au tableau d'emotion 

void initialization(int indexOfEmotion[],int sizeOfArray, int array[]) {

  for (int i=0;i<23;i++) {
    if (i<sizeOfArray) {
      array[i]=indexOfEmotion[i];       
    }
    else {
      array[i]=-1; 
    }
  }
}

//Fonction qui va afficher le pattern
//Paramètres : 
//emotionInAction : l'emotion sur lequel on est 
//patternCounter : pour savoir à quelle valeur le compteur de l'émotion on est 
//Si le patternCounter est égale à la taille du tableau, alors nous sommes à la fin de l'émotion et la fonction renvera -1, sinon elle envoie emotionInAction.pattern_number-1  

int printPattern(Emotion emotionInAction, int patternCounter, int partFace) {

  int arrayEye1[8];
  int arrayEye2[8];
  int arrayMouth1[8];
  int arrayMouth2[8];

  int testEnd;

  testEnd=emotionInAction.pattern_number-1;

  int nextIndexOfEmotion;

  for (int i=0;i<8;i++) {
    arrayEye1[i]=Eyes[emotionInAction.emotion_array_eye_1[patternCounter]][i];
  }

  for (int i=0;i<8;i++) {
    arrayMouth1[i]=MouthLeft[emotionInAction.emotion_array_mouth_1[patternCounter]][i];
  }

  for (int i=0;i<8;i++) {
    arrayMouth2[i]=MouthRight[emotionInAction.emotion_array_mouth_2[patternCounter]][i];
  }

  printOnePattern(arrayEye1,1);
  printOnePattern(arrayMouth1,3);
  printOnePattern(arrayMouth2,2);
  
  if (emotionInAction.name!="sad" and emotionInAction.name!="crazy") {
    printOnePattern(arrayEye1,4);
  }
  else {
    if (emotionInAction.name=="crazy") {
      for (int i=0;i<8;i++) {
        arrayEye2[i]=Eyes[indexOfCrazyEye2[patternCounter]][i];
      }
    }
    else {
      for (int i=0;i<8;i++) {
        arrayEye2[i]=Eyes[indexOfSadEye2[patternCounter]][i];
      }
    }
    printOnePattern(arrayEye2,4);
  }

  
  delay(emotionInAction.pattern_delay);

   if (patternCounter>=testEnd) {
     testEnd=-1;   
  }
  return testEnd;
}

//procédure qui va afficher un pattern. 
//Paramètres :
//un tableau de 8 valeurs de LED
//le nombre de la plaque de LED a controller :
//1 : Yeux gauche
//2 : Bouche gauche
//3 : Bouche gauche
//4 : Yeux droit
void printOnePattern (int value[8],int part) {
  int i;
  for (i=0;i<8;i++) {
    maxOne(part,i+1,value[i]);
  }
}

void maxAll (byte reg, byte col) {    // initialize  all  MAX7219's in the system
  int c = 0;
  digitalWrite(load, LOW);  // begin     
  for ( c =1; c<= maxInUse; c++) {
  putByte(reg);  // specify register
  putByte(col);//((data & 0x01) * 256) + data >> 1); // put data
    }
  digitalWrite(load, LOW);
  digitalWrite(load,HIGH);
}

void maxOne(byte maxNr, byte reg, byte col) {    //MaxNr = Number of the MAX  | reg = Value in the line | col | wich column ?
//maxOne is for adressing different MAX7219's, 
//whilele having a couple of them cascaded

  int c = 0;
  digitalWrite(load, LOW);  // begin     

  for ( c = maxInUse; c > maxNr; c--) {
    putByte(0);    // means no operation
    putByte(0);    // means no operation
  }

  putByte(reg);  // specify register
  putByte(col);//((data & 0x01) * 256) + data >> 1); // put data 

  for ( c =maxNr-1; c >= 1; c--) {
    putByte(0);    // means no operation
    putByte(0);    // means no operation
  }

  digitalWrite(load, LOW); // and load da shit
  digitalWrite(load,HIGH); 
}

void putByte(byte data) {
  byte i = 8;
  byte mask;
  while(i > 0) {
    mask = 0x01 << (i - 1);      // get bitmask
    digitalWrite( clock, LOW);   // tick
    if (data & mask){            // choose bit
      digitalWrite(dataIn, HIGH);// send 1
    }else{
      digitalWrite(dataIn, LOW); // send 0
    }
    digitalWrite(clock, HIGH);   // tock
    --i;                         // move to lesser bit
  }
}














