package ca.mcgill.ecse211.lab4;

//import static ca.mcgill.ecse211.lab3.Resources.US_SENSOR;
//import static ca.mcgill.ecse211.lab4.Resources.FORWARD_SPEED;
//import static ca.mcgill.ecse211.lab4.Resources.leftMotor;
//import static ca.mcgill.ecse211.lab4.Resources.rightMotor;
//import static ca.mcgill.ecse211.lab4.Resources.LCD;
import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer implements Runnable{
  private float[] usData;
  private double oldDist;
  private double newDist;
  private double firstDist;
  private double dist;
  private double last_ang;
  private double temp_ang;
  private double fall_k;
  private boolean detectInf = false;
  private boolean firstincr = true;
  double distance;
  int filterControl;
  private SampleProvider usSampler= US_SENSOR.getDistanceMode();
  
 
  /*
   * create the array containing the samples from the us sensor
   */
  public UltrasonicLocalizer(){
    usData = new float[US_SENSOR.sampleSize()];
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
  
  /**
   * the run method for the USlocalizer
   */
  public void run() {
    if (Main.buttonChoice == Button.ID_LEFT) {
      risingEdge();
    }
    else {
      fallingEdge();
    }
  }

  /**
   * Our falling edge method. The way it works is we make the robot turn right until an
   * infinite distance value is read. After infinite is read, the robot will turn
   * in increments of 4 degrees if it detects a wall. It will stop turning when it
   * is estimating itself to be perpendicular to the x-axis wall by solving x in the formula
   * newDistance/oldDistance = cos(x)/cos(x-4) since we are doing increments of 4 degrees.
   * When the robot is perpendicular to the x-axis wall, we do a 180 degree turn
   * to be at 0 degrees.
   */
  public void fallingEdge() {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION); //set acceleration
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(ROTATE_SPEED); //set speed
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward(); //make the robot turn in place clockwise
    rightMotor.backward();
    while(true) { //turn until first infinite value is read
      getDist(); //get distance from us sensor
      if(this.distance>255) { //if an infinite value is read
        detectInf=true; //set detectInf to true. this means infinite value has been read
      }
      if(detectInf && this.distance<30) { //stop turning when a distance smaller than 30 is detected after infinite value has been read. this means the x axis wall is detected
        leftMotor.stop(true); //when distance gets smaller than 30, stop motors
        rightMotor.stop(false);
        break;  // break from while loop to turn incrementally
      }
    }
    while(true) { 
      getDist(); //get distance from us sensor
      oldDist=getCleanDist();//get the average of distance before turning
      leftMotor.rotate(convertAngle(4), true); // turn in increments of 4 degrees
      rightMotor.rotate(-convertAngle(4), false);
      newDist=getCleanDist(); //calculate the average of the last 20 distance values after turning
      LCD.drawString(String.valueOf(newDist), 0, 4);
      if(firstincr) {
        temp_ang=Math.toRadians(4); //convert to radians
        last_ang=Math.abs(Math.toDegrees(Math.atan( (oldDist-newDist*Math.cos(temp_ang)) / (Math.sin(temp_ang)*newDist) ))); //last angle in degrees
        //estimated angle that the robot is relative to being perpendicular to the x axis wall
        //derived by taking the old and new distances and solving for x in this formula: newDist/oldDist=cos(x)/cos(x-4)
        firstincr=false;
        firstDist=oldDist;
        fall_k=Math.cos(Math.toRadians(last_ang-4)); //here we substract 4 degrees to get the estimated current angle
        LCD.drawString(String.valueOf(fall_k), 0, 5);
        LCD.drawString(String.valueOf(last_ang-4), 0, 6);
      }
      if(newDist<=0.86*firstDist) {
        break;
      }
    }
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.rotate(convertAngle(180), true);
    rightMotor.rotate(-convertAngle(180), false);

    while (Button.waitForAnyPress() != Button.ID_ENTER) {
    } // do nothing
    new Thread(new LightLocalizer()).start();
  }
  
  /*
   * Our rising edge method. The way it works is we make the robot turn right until an
   * infinite distance value is read. After infinite is read, the robot will turn
   * in increments of 4 degrees if it detects a wall. It will stop turning when the current distance 
   * value is bigger than 1.0045 times the last distance value, which is our
   * noise margin. (we use the average of the 20 last distances multiplied 
   * by 1.0045 as our noise margin. This means that
   * the robot is at an angle of 176 degrees counterclockwise from the positive y axis.
   * To turn the robot to 0 degrees, we make a 176 degree clockwise turn.
   */
  public void risingEdge() {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION); //set acceleration
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(ROTATE_SPEED); //set speed
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward(); //make the robot turn in place clockwise
    rightMotor.backward();
    while(true) { //turn until first infinite value is read
      getDist(); //get distance from us sensor
      if(this.distance>255) { //if an infinite value is read
        detectInf=true; //set detectInf to true. this means infinite value has been read
      }
      if(detectInf && this.distance<30) { //stop turning when a distance smaller than 30 is detected after infinite value has been read. this means the x axis wall is detected
        leftMotor.stop(true); //when distance gets smaller than 30, stop motors
        rightMotor.stop(false);
        break;  // break from while loop to turn incrementally
      }
    }
    while(true) {
      getDist(); //get distance from us sensor
      oldDist=getCleanDist(); //get the average of distance before turning
      leftMotor.rotate(convertAngle(4), true); // turn in increments of 4 degrees
      rightMotor.rotate(-convertAngle(4), false);
      newDist=getCleanDist(); //calculate the average of the last 20 distance values after turning
      if(newDist>1.0045*oldDist) {
        // cos(4)^(-1) = 1.00244, so we want it to be bigger than 1.00244 times old distance
        // we tweaked the value to 1.0045 by testing
        LCD.drawString(String.valueOf(newDist), 0, 1);
        LCD.drawString(String.valueOf(oldDist), 0, 2);
        break; //as soon as new distance is bigger than last distance, break from while loop
               // because we know we reached the 180 degree orientation + 4 degrees (so 184 degrees clockwise)
      }
    }
    leftMotor.rotate(convertAngle(176), true); //turn 176 degrees clockwise to face 0 degrees
    rightMotor.rotate(-convertAngle(176), false);
    while (Button.waitForAnyPress() != Button.ID_ENTER) {
    } // do nothing as long as enter button is not pressed
    new Thread(new LightLocalizer()).start(); //start the light localizer thread after enter button is pressed
  }
  
   /*
   * fetch distance from sensor and convert it to centimeters
   */
  double getDist() {
    usSampler.fetchSample(usData, 0); // acquire distance data in meters
    dist = (double) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
    filter(dist);
    return this.distance;
  }

  /*
   * This method is to get the mean of the last 20 distance values read by the sensor.
   * By calculating the average, we increase the precision
   */
  double getCleanDist() {
    double sumDist=0;
    int bigly=20; //use last 20 distance values
    for(int i=0; i<bigly; i++) {
      sumDist+=getDist(); //add values to find mean
      try {
        Thread.sleep(10);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
    return sumDist/bigly; //calculate mean
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * @param distance distance in cm
   */
  void filter(double distance) {
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the filter value
      filterControl++;
    } else if (distance >= 255) {
      // Repeated large values, so there is nothing there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      this.distance = distance;
    }
  }
  
}
