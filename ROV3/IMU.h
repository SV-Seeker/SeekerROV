#define SERIAL1RATE 57600     //serial rate to IMU

float YPR[6];              //variable to hold the yaw-pitch-roll readings from the IMU (positions then rates)

void rawReadArr(void * varr, uint8_t arr_length) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) arr[i]=((byte)Serial1.read());
}

void ResetIMU(int delaytime){
  Serial1.end();                    //turn off the Serial1
  digitalWrite(IMURESETPIN, HIGH);  //hold the IMU reset line low, resetting the IMU
  delay(delaytime/2);               //wait for the IMU to recognize the reset command
  digitalWrite(IMURESETPIN, LOW);   //let the IMU start up
  delay(delaytime/2);               //let the IMU start up
  Serial1.begin(SERIAL1RATE);             //turn the Serial1 back on
  Serial1.print('y');               //request the first roll-pitch-yaw reading from the IMU.
}


int IMUcommunicate(int retrying){
  int gotNewData = 0;
  int YPRarraySize = 6*sizeof(float);
  
  //
  if(Serial1.available() >= YPRarraySize+3) {
    if (Serial1.read()=='Y') {          //each new yaw-pitch-roll reading starts with the character 'Y', so look for that to be ready in the serial recieve buffer
      rawReadArr(YPR, YPRarraySize);
      Serial1.read();                   //discard newline
      Serial1.read();                   //discard linefeed
      Serial1.print('y');               //request a new reading for the next time around
      constrain(YPR[0], -180.0, 180.0);
      constrain(YPR[3], -180.0, 180.0);
      constrain(YPR[1], -90.0, 90.0);
      constrain(YPR[4], -180.0, 180.0);
      gotNewData=1;                     //note that new data was recieved
    }
    else {                                          //if there is serial data available from the IMU but it doesn't start with a 'Y' we don't know what to do with it
      for (int i=0; i<YPRarraySize+2; i++) {        //so throw it away
        if(!Serial1.available()) break;             //(but don't keep reading and tossing if there's no more data available)
        if(Serial1.read()=='\n') {                  //stop tossing when the line ends
          if(Serial1.peek()=='\r') Serial1.read();  //read the expected linefeed afterward
          break; 
        }
      }
    }
  } 
  if (retrying && !gotNewData) {                 //if we haven't recieved a good reading in a while
    while (Serial1.available()) Serial1.read();  //toss any remaining scraps
    Serial1.println('y');                        //and ask for more data again
  }
  // 
  return gotNewData;
}
