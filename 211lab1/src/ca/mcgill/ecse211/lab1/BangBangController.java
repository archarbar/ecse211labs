//Victor Zhong 260865384
//Panayiotis Demopoulos 260805660
package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class BangBangController extends UltrasonicController {

  public BangBangController() {
    LEFT_MOTOR.setSpeed(MOTOR_HIGH); // Start robot moving forward
    RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);
    
    //if in the middle, go straight
    if(BAND_CENTER-BAND_WIDTH < this.distance && this.distance < BAND_CENTER+BAND_WIDTH) {
      LEFT_MOTOR.setSpeed(MOTOR_HIGH); // Start robot moving forward
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
    //else if too close, go right to increase distance
    else if (BAND_CENTER-BAND_WIDTH >= this.distance) {
      if(this.distance>12) {
        LEFT_MOTOR.setSpeed((int)1.8*MOTOR_HIGH); // turn right, This multiplied to make the right turns sharper than the left turns
        RIGHT_MOTOR.setSpeed(MOTOR_LOW);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
      }
      else { //If distance from wall is less than 12cm
          for(int i=0; i<7000; i++) {
            LEFT_MOTOR.setSpeed(160); // back up to avoid collision
            RIGHT_MOTOR.setSpeed(160);
            LEFT_MOTOR.backward();
            RIGHT_MOTOR.backward();
          }
          for(int i=0;i<7000;i++) {
            LEFT_MOTOR.setSpeed(170); // turn right in place after backup to correct path
            RIGHT_MOTOR.setSpeed(170);
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.backward();
          }
          for(int i=0;i<7000;i++) {
            LEFT_MOTOR.setSpeed(150); // move forward slightly to avoid being too close to the wall, due to another sharp left turn being likely, also prevents getting in a backup-turn loop 
            RIGHT_MOTOR.setSpeed(150);
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.forward();
          }
       }
    }
    // else, means distance is too big, so turn left
    else {
      LEFT_MOTOR.setSpeed(MOTOR_LOW); // turn left
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
