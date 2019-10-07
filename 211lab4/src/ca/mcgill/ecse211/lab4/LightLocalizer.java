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
    findDistance();
  }
  private void findDistance() {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    
    while (true) {
      colorSampleProvider.fetchSample(csData, 0); //get data from sensor
      if(csData[0] < LINE_RED_INTENSITY) { //if light read by sensor is smaller (darker) than red light, eg., black lines
        Sound.beep(); // sound alert to know when the sensor hits a line
        leftMotor.stop(true);
        rightMotor.stop(true);
        leftMotor.rotate(convertDistance(-5), true);
        rightMotor.rotate(convertDistance(-5), false);
        leftMotor.rotate(convertAngle(90.0), true);
        rightMotor.rotate(-convertAngle(90.0), false);
        leftMotor.forward();
        rightMotor.forward();
        if(csData[0] < LINE_RED_INTENSITY) { //if light read by sensor is smaller (darker) than red light, eg., black lines
          Sound.beep();
          leftMotor.rotate(-convertAngle(90.0), true);
          rightMotor.rotate(convertAngle(90.0), false);
          leftMotor.rotate(convertDistance(2), true);
          rightMotor.rotate(convertDistance(2), false);
        }
      }
    }

  }
}
