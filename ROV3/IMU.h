Adafruit_BNO055 bno = Adafruit_BNO055();

imu::Vector<3> YPR;                 //global variable to hold Yaw Pitch Roll data
imu::Vector<3> dYPR;                //global variable to hold the Yaw Pitch and Roll derivatives/rates

//For the Yaw Pitch Roll data to make sense, the board needs to be mounted in the ROV right-side-up and facing forward.

void ResetIMU(int delaytime){
  digitalWrite(IMURESETPIN, LOW);   //hold the IMU reset line low, resetting the IMU
  delay(delaytime/2);               //wait for the IMU to recognize the reset command
  digitalWrite(IMURESETPIN, HIGH);  //let the IMU start up
  delay(delaytime/2);               //let the IMU start up
}


int IMUcommunicate(int retrying){
  YPR = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  dYPR = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return 1;
}
