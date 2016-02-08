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
#include <Canutil.h>
#include <MCP2510.h>
// Defining constants
#define dt 20                       // time difference in milli seconds
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.95             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)



//id LSB = priorité, si 0 prioritaire,  15 bits suivant id hardware (1), et apres 8bits id messages
#define IdAlertesecousse  0b000000000000000100000001      // 0, 1, 1
#define IdAlerteretourner 0b000000000000000100000010     //0, 1, 2
#define IdAlerteChute 0b000000000000000100000011        //0, 1, 3
//#define IdAlerteChoc 0b000000000000000100000100        //0, 1, 4

#define IDangleFromNorth 0b100000000000000100000101 //0, 1, 5
//#define IDangleFromCheckPoint 0b100000000000000100000110      //0, 1, 6
#define IDvitesseXYZ 0b100000000000000100000111        //0, 1, 7
#define IDpositionXYZ 0b100000000000000100001000     //0, 1, 8
#define IDangle 0b100000000000000100001001   //0, 1, 9 


#define addr 0x1E //I2C Address for The HMC5883

#define node_id 0x003
#define shock_alert_id 0x20001
#define turned_alert_id 0x20002
#define fall_alert_id 0X20003


// *********************************************************************
//    Global Variables
// *********************************************************************
unsigned long t = 0; // Time Variables
float angle_x_gyro = 0, angle_y_gyro = 0, angle_z_gyro = 0, angle_x_accel = 0, angle_y_accel = 0, angle_z_accel = 0, angle_x = 0, angle_y = 0, angle_z = 0;
float vitesse_x = 0, vitesse_y = 0, vitesse_z = 0, position_x = 0, position_y = 0, position_z = 0;
MCP2510  can_dev (9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);
volatile uint16_t msgID;
volatile int recSize;
volatile uint8_t canDataReceived[8];
uint8_t txstatus;
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

  can_dev.write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
  can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

  canutil.setClkoutMode(0, 0); // disables CLKOUT
  canutil.setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input

  canutil.setOpMode(4); // sets configuration mode
  // IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
  // filters and acceptance masks can be modified

  canutil.waitOpMode(4);  // waits configuration mode

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

  int x, y, z; //triple axis data
  float angleNorth;
  int machine = 0;


  switch (machine) {

    case 0 :
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

    case 1 :
      angle_x_gyro = (gyro_x_scalled * ((float)dt / 1000) + angle_x);
      angle_y_gyro = (gyro_y_scalled * ((float)dt / 1000) + angle_y);
      angle_z_gyro = (gyro_z_scalled * ((float)dt / 1000) + angle_z);

      angle_z_accel = atan(accel_z_scalled / (sqrt(accel_y_scalled * accel_y_scalled + accel_x_scalled * accel_x_scalled))) * (float)rad2degree;
      angle_y_accel = -atan(accel_y_scalled / (sqrt(accel_y_scalled * accel_y_scalled + accel_z_scalled * accel_z_scalled))) * (float)rad2degree;
      angle_x_accel = atan(accel_x_scalled / (sqrt(accel_x_scalled * accel_x_scalled + accel_z_scalled * accel_z_scalled))) * (float)rad2degree;

      angle_x = Filter_gain * angle_x_gyro + (1 - Filter_gain) * angle_x_accel;
      angle_y = Filter_gain * angle_y_gyro + (1 - Filter_gain) * angle_y_accel;
      angle_z = Filter_gain * angle_z_gyro + (1 - Filter_gain) * angle_z_accel;

      //////////Tester véracité de la vitesse et de la position

      vitesse_x = (accel_x_scalled * ((float)dt / 1000) + vitesse_x);
      vitesse_y = (accel_y_scalled * ((float)dt / 1000) + vitesse_y);
      vitesse_z = (accel_z_scalled * ((float)dt / 1000) + vitesse_z);

      position_x = (vitesse_x * ((float)dt / 1000) + position_x);
      position_y = (vitesse_y * ((float)dt / 1000) + position_y);
      position_z = (vitesse_z * ((float)dt / 1000) + position_z);

      angleNorth = (round(atan2(x, y) * 180 / 3.14159265) - 90) * 2;

      if (accel_z_scalled < 4 || accel_x_scalled > 15 || accel_y_scalled > 15 || accel_z_scalled > 15) {
        machine = 3;
      } else {
        machine = 2;
      }
      break;



    case 2 :
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


    case 3 :
      //TODO
      //Envoi du message d'Alerte: secousse
      //donnée: d'accélération, 2 float (accel_x_scalled ou accel_y_scalled ou accel_z_scalled)

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

    case 4 :
      //TODO
      //Envoi du message d'Alerte: robot retourné
      //donnée: angle, 2 float (angle_x et angle_y)

      // #define IdAlerteretourner 0b000000000000000100000010
      canutil.setTxBufferID(node_id, turned_alert_id, 1, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
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


    case 5 :
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


    case 10 :
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

    case 1000 :


      //TODO

      //état de lecture du CAN pour recevoir les information de la raspberry
      //si il y a demande d'info, machine =0;


      break;
  }



  while ((millis() - t) < dt) { // Making sure the cycle time is equal to dt
    // Do nothing
  }


  delay(500);
}








