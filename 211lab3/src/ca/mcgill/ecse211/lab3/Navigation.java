package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import ca.mcgill.ecse211.lab3.Odometer;

public class Navigation extends Thread {
  
  private Odometer odometer;
  public Navigation() {
    this.odometer = Odometer.getOdometer();
  }

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
   * The run method
   */
  public void run() {
    // reset the motors
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    odometer.setX(TILE_SIZE);
    odometer.setY(TILE_SIZE);
    odometer.setTheta(0);
    // travel to coordinates
    travelTo(3, 2);
    travelTo(2, 2);
    travelTo(2, 3);
    travelTo(3, 1);
  }
  
  /**
   * A method to drive our vehicle to a certain cartesian coordinate
   * 
   * @param x X-Coordinate
   * @param y Y-Coordinate
   */
  private void travelTo(double x, double y) {
    double currentPosition[] = odometer.getXYT();
    double currentX = currentPosition[0]; //get current x position
    double currentY = currentPosition[1]; //get current y position
    isNavigating = true;
    double deltaX = x*TILE_SIZE - currentX;
    double deltaY = y*TILE_SIZE - currentY;

    // calculate the minimum angle
    double firstTheta = Math.atan2(deltaX, deltaY);

    // turn to the minimum angle
    turnTo(firstTheta);

    // calculate the distance to next point
    double distance  = Math.hypot(deltaX, deltaY);

    // move to the next point
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
    leftMotor.stop(true);
    rightMotor.stop(true);
    isNavigating = false;
  }

  /**
   * A method to turn our vehicle to a certain angle
   * 
   * @param theta
   */
  private void turnTo(double theta) {
    double currentTheta = odometer.getXYT()[2]; // get current theta angle in radians
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
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
  private double getMinAngle(double angle) {
    if (angle > Math.PI) {
      angle -= 2*Math.PI;
    }
    if (angle < -(Math.PI)) {
      angle += 2*Math.PI;
    }
    return angle;
  }

  /**
   * A method to determine whether another thread has called travelTo and turnTo methods or not
   * 
   * @return
   */
  private boolean isNavigating() {
    return false; // TODO
  }


}