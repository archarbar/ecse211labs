package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class LightLocalizer implements Runnable{
  private float[] csData;
  private SampleProvider colorSampleProvider= colorSensor.getRedMode(); // use a red light to compare luminence level
  private static final float LINE_RED_INTENSITY = (float) 0.3; // cast to float since default is double
  
  public LightLocalizer() {
    csData = new float[colorSensor.sampleSize()];
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
  
  public void run() {
    moveToDestination();
  }
  /**
   * Since the robot is already oriented at 0 degrees, make it go forward
   * until it reaches a line. This is the horizontal line of the destination.
   * Back up 5 centimeters to know the vertical distance to travel to reach destination,
   * make a right turn and go forward until a line is reached. This is the vertical 
   * line of destination. Make a left turn and go forward by 5 centimeters and 
   * destination is reached.
   */
  private void moveToDestination() {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED); //set speed to forward speed
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward(); //move forward until line is reached
    rightMotor.forward();
    
    while (true) {
      colorSampleProvider.fetchSample(csData, 0); //get data from sensor
      if(csData[0] < LINE_RED_INTENSITY) { //if light read by sensor is smaller (darker) than red light, eg., black lines
        leftMotor.stop(true); //stop the robot when a line is detected
        rightMotor.stop(false);
        Sound.beep(); // sound alert to know when the sensor hits a line
        leftMotor.rotate(convertDistance(-5), true); //make the robot back up 5 centimeters so we know its vertical distance to destination
        rightMotor.rotate(convertDistance(-5), false);
        leftMotor.rotate(convertAngle(90.0), true); //make a right turn
        rightMotor.rotate(-convertAngle(90.0), false);
        break;
      }
    }
    while(true) {
      colorSampleProvider.fetchSample(csData, 0); //get data from sensor
      leftMotor.forward(); //move forward until a black line is encountered
      rightMotor.forward();
      if(csData[0] < LINE_RED_INTENSITY) { //if light read by sensor is smaller (darker) than red light, eg., black lines
        leftMotor.stop(true); //stop the motors when a line is detected
        rightMotor.stop(false);
        Sound.beep();
        leftMotor.rotate(-convertAngle(90.0), true); //make a left turn
        rightMotor.rotate(convertAngle(90.0), false);
        leftMotor.rotate(convertDistance(5), true); //go forward 5 centimeters to reach to destination
        rightMotor.rotate(convertDistance(5), false);
        break;
      }
    }
  }
}
