//libraries used
#include <Servo.h>
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//other files in this sketch
#include "pins.h"
#include "IMU.h"
#include "ROVethernet.h"
#include "escs.h"

#define IMUCOMMTIMEOUT 2000   //if the imu hasn't responded in this many ms, retry to get data from it
#define CMDTIMEOUT 2000       //how many ms to wait for a command from remote control before we start to worry
#define REPORTINTERVAL 103    //report the current status to the PC once in this many ms
#define IMUINTERVAL 31        //update the current YPR after this many ms
#define SONINTERVAL 53        //update the altitude after this many ms
#define SONARDECAY 0.99       //exponential decay rate for the sonar watchdog
#define IMUDECAY 0.99         //exponential decay rate for the IMU watchdog
#define COMMANDDECAY 0.99999  //exponential decay rate for PC commands
#define SERIALRATE 57600      //serial rate to PC controller

float IMUwatchdog = 1.0;   //gain on the IMU control inputs that slowly fades down to zero if the IMU goes MIA
float SONwatchdog = 1.0;   //gain on the sonar altitude control inputs that slowly fades down to zero if the sonar goes MIA
float PCwatchdog =  1.0;   //gain on the command inputs that slowly fades down to zero if the PC hasn't communicated in a while

int LastCommandTime = 0;   //last millis() reading when a command from the pilot was received

void setup() {
  setupOutputPins();                //set all the outputs to be outputs and initialize most of them
  initializeESCs(6500);             //initialize the ESCs and wait for them to arm
  Serial.begin(SERIALRATE);         //start the serial connection to the PC
  Serial1.begin(SERIAL1RATE);       //start the serial connection to the arduIMU
  Ethernet.begin(mac, ip);          //connect the ethernet shield
  Udp.begin(localPort);             //start the UDP interface
}




void loop() {
  static unsigned long EventTimer;
  static unsigned long NextReport  = 7000;      // first timed events happen ~7 seconds after powerup
  static unsigned long NextIMUread = 7000;      //   /
  static unsigned long NextSONread = 7000;      //  /
  
  EventTimer = millis();                      //get the current time, to see what all needs to happen
  
  curPacketSize=Udp.parsePacket();       //check to see if new data is available from the pilot
  if (curPacketSize) {                   //if so,
    if(UpdateCommand()!='0'){                 //process new commands
      PCwatchdog=1.0;                           //keep the watchdog saturated
      LastCommandTime = EventTimer;             //note the time
    }
    else {
      if ((EventTimer-LastCommandTime)>CMDTIMEOUT) {
        PCwatchdog *= COMMANDDECAY;             //after a while, start forgetting the pilot's last command
      }
    }
  }
  
  
  //don't bother the sonar very often, altitude changes slowly
  if (EventTimer>NextSONread) {
    NextSONread += SONINTERVAL;
    if (SonarCommunicate()) SONwatchdog = 1.0;   //if we get new data from the sonar, keep controlling based on its output
    else SONwatchdog *= SONARDECAY;              //otherwise slowly start pretending it's not there
  }
  
  //don't bother the IMU too often, it has a lot to do
  if (EventTimer>NextIMUread) {               //if it's time to get new data from the IMU
    NextIMUread = (NextIMUread+IMUINTERVAL);                   //schedule the next one
      /*this next line checks to see if the IMUwatchdog is less than 0.8, which means
        it's been a while since the IMU communicated. if that is the case, a 'true'
        value is passed into the IMUcommunicate() function, which tells it to re-request
        data from the IMU. This is just in case communications with the IMU have gotten out-of-sync     */
    if (IMUcommunicate((IMUwatchdog<0.8))) IMUwatchdog=1.0;    //if we get new data from the IMU, keep controlling based on its output
    else IMUwatchdog *= IMUDECAY;                              //if we don't get data from the IMU, slowly start pretending it's not there
    YawControl();              //set the outputs for the yaw motors, trying to satisfy the external commands and the internal controls
    PitchControl();            //set the outputs for the pitch motors
    DriveControl();            //set the outputs for the drive motors
  }
  
  
  
  if (EventTimer>NextReport) {                //if it's time for another status report,
    NextReport += REPORTINTERVAL;               //schedule the next one
    udpReport();                                //and do the report
    //serialReport();
  }
}




void udpReport() {
  char tempc[16];
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write('Y');
  Udp.write(itoa(round(YPR[0]*100), tempc, 10));
  Udp.write(', P');
  Udp.write(itoa(round(YPR[1]*100), tempc, 10));
  Udp.write(', R');
  Udp.write(itoa(round(YPR[2]*100), tempc, 10));
  Udp.endPacket();
}



void serialReport() {
  Serial.print(YPR[0]);
  Serial.print("        ");
  Serial.print(YPR[1]);
  Serial.print("        ");
  Serial.print(YPR[2]);
  Serial.println("        ");
}



int SonarCommunicate(){
  return 0;
}






void YawControl(){
  const float Kpr = 4.0;
  const float Kde = 12.0;
  const float Kin = 0.4/1000.0; //divide by 1000 because time is in ms
  const float integralSaturation = 100.0/Kin; //numerator is the max that we want the contribution to be AFTER multiplying by Kin
  static float integratedErr = 0.0;
  static int oldcmd;
  int cmd;
  float err = (YPR[0]-HdgCommand)*IMUwatchdog*PCwatchdog;
  integratedErr += err*IMUINTERVAL;
  if (integratedErr > integralSaturation) integratedErr = integralSaturation;
  else if (integratedErr < -integralSaturation) integratedErr = -integralSaturation;
  if(err < -180.0) err+=360.0;
  else if (err > 180.0) err-=360.0;
  cmd = round(Kpr*err - Kde*YPR[3] + Kin*integratedErr);
  if(cmd>0) cmd+=DEDZONESC;
  else cmd-=DEDZONESC;
  cmd = constrain(cmd, -YAWSATURATION, YAWSATURATION);
  if(cmd>(oldcmd+SLEWYAW)) cmd=oldcmd+SLEWYAW;
  else if (cmd<(oldcmd-SLEWYAW)) cmd=oldcmd-SLEWYAW;
  oldcmd=cmd;
  YawBow.write(OFFSIGESC-cmd);
  YawStern.write(OFFSIGESC+cmd);
}




void PitchControl(){
  const float Kpr = 8.0;
  const float Kde = 18.0;
  const float Kin = 10.0/1000.0; //divide by 1000 because time is in ms
  const float integralSaturation = 250.0/Kin; //numerator is the max that we want the contribution to be AFTER multiplying by Kin
  static float integratedErr = 0.0;
  static int oldcmd;
  int cmd;
  float err = (YPR[1]-PchCommand)*IMUwatchdog*PCwatchdog;
  integratedErr += err*IMUINTERVAL;
  if (integratedErr > integralSaturation) integratedErr = integralSaturation;
  else if (integratedErr < -integralSaturation) integratedErr = -integralSaturation;
  cmd = round(Kpr*err - Kde*YPR[4] + Kin*integratedErr);
  if(cmd>0) cmd+=DEDZONESC;
  else cmd-=DEDZONESC;
  cmd = constrain(cmd, -PITCHSATURATION, PITCHSATURATION);
  if(cmd>(oldcmd+SLEWPITCH)) cmd=oldcmd+SLEWPITCH;
  else if (cmd<(oldcmd-SLEWPITCH)) cmd=oldcmd-SLEWPITCH;
  oldcmd=cmd;
  PitchBow.write(OFFSIGESC-cmd);
  PitchStern.write(OFFSIGESC+cmd);
}




void DriveControl(){
}
