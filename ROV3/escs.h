#define MINSIGESC 1000
#define MAXSIGESC 2000
#define OFFSIGESC 1500
#define DEDZONESC 10
#define SLEWYAW 80
#define SLEWPITCH 80
#define YAWSATURATION 300
#define PITCHSATURATION 300

Servo PitchBow;       //create six global servo objects which will control the ESCs
Servo PitchStern;
Servo YawBow;
Servo YawStern;
Servo DrivePort;
Servo DriveStarb;

void initializeESCs(int delaytime) {
  PitchBow.attach(PITCHBOWPIN, MINSIGESC, MAXSIGESC);          //attach ESC controls to their output pins
  PitchStern.attach(PITCHSTERNPIN, MINSIGESC, MAXSIGESC);      //  |
  YawBow.attach(YAWBOWPIN, MINSIGESC, MAXSIGESC);              //  |
  YawStern.attach(YAWSTERNPIN, MINSIGESC, MAXSIGESC);          //  |
  DrivePort.attach(DRIVEPORTPIN, MINSIGESC, MAXSIGESC);        //  |
  DriveStarb.attach(DRIVESTARBPIN, MINSIGESC, MAXSIGESC);      // /
  
  PitchBow.write(OFFSIGESC);
  PitchStern.write(OFFSIGESC);
  YawBow.write(OFFSIGESC);
  YawStern.write(OFFSIGESC);
  DrivePort.write(OFFSIGESC);
  DriveStarb.write(OFFSIGESC);
  delay(delaytime);
}
