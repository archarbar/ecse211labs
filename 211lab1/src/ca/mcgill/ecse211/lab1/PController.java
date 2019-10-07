//Panyiotis Demopoulos 260805660
//Victor Zhong

package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class PController extends UltrasonicController {

  private static final int MOTOR_SPEED = 140;

  public PController() {
    LEFT_MOTOR.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }
  
  @Override
  public void processUSData(int distance) {
    filter(distance);
    int DIST_ERROR = 0;
    int MAX_SPEED = 350; //maximum right-wheel speed during left turn
    int MIN_SPEED = 115; //minimum left-wheel speed during left turn
    DIST_ERROR = BAND_CENTER - this.distance; //To see how far we are from our desired distance
    int correction = Math.abs(DIST_ERROR * 6); //fine tuning: multiply by 6 to scale correction value
    
    // if error is within bounds, go straight
    if (Math.abs(DIST_ERROR) <= BAND_WIDTH) {
      LEFT_MOTOR.setSpeed(MOTOR_SPEED); // robot moving forward
      RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
    //else if error is positive and bigger than error margin, move away from wall
    else if (DIST_ERROR > BAND_WIDTH) {
      if (DIST_ERROR<28) { //if distance from wall is greater than 12cm, simply turn right
        LEFT_MOTOR.setSpeed(MOTOR_SPEED + (int)1.7*correction); //we multiply since sharpening our right turns compensates for poor left turns
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED - correction);
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
    
    //else go left
    else {
      if (MOTOR_SPEED + correction/3 > MAX_SPEED) { //limit left turn speed
        RIGHT_MOTOR.setSpeed(MAX_SPEED);
      }
      else {
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED + correction/3); //fine tuning: we divide by 3 to avoid our left turns being too sharp
      }
      if (MOTOR_SPEED - correction/2 < MIN_SPEED) { //limit left turn speed
        LEFT_MOTOR.setSpeed(MIN_SPEED);
      }
      else {
        LEFT_MOTOR.setSpeed(MOTOR_SPEED - correction/2); //fine tuning: divide by 2 to reduce correction
      }
      RIGHT_MOTOR.forward();
      LEFT_MOTOR.forward();
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
