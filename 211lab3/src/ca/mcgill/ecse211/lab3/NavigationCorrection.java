package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import ca.mcgill.ecse211.lab3.Odometer;
import lejos.hardware.Sound;

public class NavigationCorrection implements Runnable {
  private float[] usData;
  private int dist;
  private Odometer odometer;
  int distance;
  int filterControl;
  
  public NavigationCorrection() {
    this.odometer = Odometer.getOdometer();
    usData = new float[US_SENSOR.sampleSize()];
  }

  //set isNavigating to true
  private static boolean isNavigating = true;
  
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
   * The run method, calls travelTo to make the robot travel to coordinates
   */
  public void run() {
    // reset the motors
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    // set odometer starting values to coordinates (1, 1)
    odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
    // travel to coordinates
    travelTo(1, 3);
    travelTo(2, 2);
    travelTo(3, 3);
    travelTo(3, 2);
    travelTo(2, 1);
  }
  
  /**
   * A method to drive the robot to a specific cartesian coordinate
   * 
   * @param x X-Coordinate
   * @param y Y-Coordinate
   */
  private void travelTo(double x, double y) {
    double currentPosition[] = odometer.getXYT();
    double currentX = currentPosition[0]; //get current x position
    double currentY = currentPosition[1]; //get current y position
    isNavigating = true; //robot is navigating, set the boolean to true
    double deltaX = x*TILE_SIZE - currentX;
    double deltaY = y*TILE_SIZE - currentY;
    double posErr = 2.5;

    // calculate the minimum angle 
    double firstTheta = Math.atan2(deltaX, deltaY);

    // turn to the minimum angle
    turnTo(firstTheta);

    
    // move to the next point
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.forward(); //used forward instead of "rotate(convertDistance)" so we can cancel the motion when the robot is too close to an object
    rightMotor.forward();
    
//      Sensors now return floats using a uniform protocol. Need to convert US result to an integer [0,255] (non-Javadoc)
//      @see java.lang.Thread#run()
    while (leftMotor.isMoving() || rightMotor.isMoving()) { //while a motor is moving, fetch distance data from sensor and react accordingly
      US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire distance data in meters
      dist = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
      filter(dist);
      if (this.dist < 10) { //if robot is closer that 10 centimeters to object, avoid it
        leftMotor.stop(true);
        rightMotor.stop(true);
        // turn 90 degrees right first
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        leftMotor.rotate(convertAngle(90.0), true);
        rightMotor.rotate(-convertAngle(90.0), false);
        // drive forward one tile
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        
        leftMotor.rotate(convertDistance(TILE_SIZE), true);
        rightMotor.rotate(convertDistance(TILE_SIZE), false);
        Sound.beep();
        // turn 90 degrees left second
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        rightMotor.rotate(convertAngle(90.0), true);
        leftMotor.rotate(-convertAngle(90.0), false);
        
        // drive forward one tile
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);

        leftMotor.rotate(convertDistance(TILE_SIZE), true);
        rightMotor.rotate(convertDistance(TILE_SIZE), false);
        travelTo(x, y); // return to initial point after avoiding object
      }
      double Position[] = odometer.getXYT();
      double Xpos = Position[0]; //get current x position
      double Ypos = Position[1]; //get current y position
      if (Math.abs(x*TILE_SIZE-Xpos) < posErr //if the distance between current position and destination is smaller than an error value, we have arrived destination
              && Math.abs(y*TILE_SIZE-Ypos) < posErr) { 
          break; //exit from while loop so robot can stop, since it has arrived destination
      }
    }

    leftMotor.stop(true);
    rightMotor.stop(true);
    isNavigating = false; //robot has arrive to destination, set boolean to false
  }

  /**
   * method that makes the robot turn to a specific angle
   * 
   * @param theta
   */
  private void turnTo(double theta) {
    double currentTheta = odometer.getXYT()[2]; // get current theta angle in radians
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    //calculate minimum theta
    double minTheta = getMinAngle(theta - Math.toRadians(currentTheta));
    
    if(minTheta < 0) { // if angle is negative, turn to the left
      leftMotor.rotate(-convertAngle(-(minTheta*180)/Math.PI), true);
      rightMotor.rotate(convertAngle(-(minTheta*180)/Math.PI), false);
    }
    else { // angle is positive, turn to the right
      leftMotor.rotate(convertAngle(minTheta*180/Math.PI), true);
      rightMotor.rotate(-convertAngle(minTheta*180/Math.PI), false);
    }
  }
  
  /*
   * method that calculates the minimum angle (between -pi and pi)
   */
  private double getMinAngle(double angle) {
    if (angle > Math.PI) { //if the angle is bigger than pi, decrease it by 2 pi
      angle -= 2*Math.PI;
    }
    if (angle < -(Math.PI)) { //if the angle is smaller that -pi, increase it by 2pi
      angle += 2*Math.PI;
    }
    return angle;
  }

  /**
   * method to determine if a thread has called travelTo or turnTo methods or not
   * 
   * 
   */
  private boolean isNavigating() {
    return false;
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * @param distance distance in cm
   */
  void filter(int distance) {
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