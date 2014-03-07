//pin definitions for ROV

//obviously:
#define LEDPIN 13

//power relays for big lights
#define LIGHTPOWPIN0 2
#define LIGHTPOWPIN1 3
#define LIGHTPOWPIN2 4
#define LIGHTPOWPIN3 5

//motor control pins
#define PITCHBOWPIN 22
#define PITCHSTERNPIN 23
#define YAWBOWPIN 24
#define YAWSTERNPIN 25
#define DRIVEPORTPIN 26
#define DRIVESTARBPIN 27

//pin to reset IMU
#define IMURESETPIN 39

//pins to turn on direct power to motor controllers
#define BOWPOWPIN1 30
#define STERNPOWPIN1 31
#define BOWPOWPIN2 32
#define STERNPOWPIN2 33
#define ESCPOWPIN4 34
#define ESCPOWPIN5 35
#define ESCRELAYPORT PORTC

//pin to turn on power to the sonar
#define SONARPOWPIN 40

//pin to turn on power to the cameras
#define CAMPOWPIN 41

void setupOutputPins(){
  pinMode(IMURESETPIN, OUTPUT);    //get output control of the reset pin on the arduIMU
  pinMode(LEDPIN, OUTPUT);         //LED is an output
  pinMode(LIGHTPOWPIN0, OUTPUT);   //pins for light power relays are outputs
  pinMode(LIGHTPOWPIN1, OUTPUT);   //  |
  pinMode(LIGHTPOWPIN2, OUTPUT);   //  |
  pinMode(LIGHTPOWPIN3, OUTPUT);   // /
  pinMode(BOWPOWPIN1, OUTPUT);     //pins for ESC power relays are outputs
  pinMode(STERNPOWPIN1, OUTPUT);     //  |
  pinMode(BOWPOWPIN2, OUTPUT);     //  |
  pinMode(STERNPOWPIN2, OUTPUT);     //  |
  pinMode(ESCPOWPIN4, OUTPUT);     //  |
  pinMode(ESCPOWPIN5, OUTPUT);     // /
  pinMode(SONARPOWPIN, OUTPUT);    //pin for sonar power relay is an output
  pinMode(CAMPOWPIN, OUTPUT);      //pin for camera power relay is an output
  
  digitalWrite(LIGHTPOWPIN0, HIGH);  //start with lights off
  digitalWrite(LIGHTPOWPIN1, HIGH);  //  |
  digitalWrite(LIGHTPOWPIN2, HIGH);  //  |
  digitalWrite(LIGHTPOWPIN3, HIGH);  // /
  digitalWrite(BOWPOWPIN1, LOW);    //start with motors off
  digitalWrite(STERNPOWPIN1, LOW);    //  |
  digitalWrite(BOWPOWPIN2, HIGH);    //  |
  digitalWrite(STERNPOWPIN2, HIGH);    //  |
  digitalWrite(ESCPOWPIN4, HIGH);    //  |
  digitalWrite(ESCPOWPIN5, HIGH);    //  /
  digitalWrite(SONARPOWPIN, HIGH);   //start with sonar off
  digitalWrite(CAMPOWPIN, HIGH);     //start with cameras off
  //digitalWrite(IMURESETPIN, HIGH);  //don't know whether high or low resets...
}
