package ca.mcgill.ecse211.lab5;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import static ca.mcgill.ecse211.lab5.Resources.*;


public class LightLocalizer {

  // the SENSOR_OFFSET represents the distance between the center of the wheelbase and the light sensor
  private final double SENSOR_OFFSET = 13;


  private static int ROTATE_SPEED = 50;
  private static int MOTOR_SPEED = 100;

  // threshold that will be used for the light sensor values
  private static double initValue;
  // private static final double lineThreshold = 185;
  private static double lineThreshold;

  private int lineCounter = 0;
  double[] ThetaArray = new double[4];


  private Odometer odometer;


  private EV3LargeRegulatedMotor leftMotor, rightMotor;


  // private static final Port lsPort = LocalEV3.get().getPort("S1");
  // EV3ColorSensor colSensor = new EV3ColorSensor(lsPort);
  SampleProvider lsValue;
  private float[] lsData;



  private double TRACK;
  private double WHEEL_RAD;


  /**
   * call dependencies needed for light localization
   * 
   * @param leftMotor
   * @param rightMotor
   * @param odometer
   * @param TRACK
   * @param WHEEL_RAD
   */
  public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
      double TRACK, double WHEEL_RAD) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    centerLightSensor.setCurrentMode("Red");
    this.lsValue = centerLightSensor.getMode("Red");
    this.lsData = new float[lsValue.sampleSize()];
    initValue = getIntensity();
    lineThreshold = 0.85 * initValue;
  }

  /**
   * This is the main method that is called upon in the main. This method makes the robot travel to the origin, apply a
   * correction algorithm, and stop at origin in correct angle finally.
   * 
   * it starts by orienting the robot towards the origin, it then makes the robot travels to the origin. Once the robot
   * reaches the origin(by detecting a black line), it makes it stop then move backwards by the SENSOR_OFFSET value.
   * Then it will make the robot rotate clockwise until it detects 4 black lines, and store the robot's theta value each
   * time it crosses one. Then by a small series of calculations(detailed below) the robot's position relative to the
   * origin is found and we can now make the robot travel to the origin. Finally all that is left to do is to correct
   * the heading of the robot, this is done by reading the angle reported by the odometer and make the robot rotate by
   * the negative of this value, this way the robot will be on the 0-degree axis.
   * 
   * 
   */
  public void mainMethod() {


    turnTo(45);



    while (getIntensity() > lineThreshold) {
      leftMotor.setSpeed(MOTOR_SPEED);
      rightMotor.setSpeed(MOTOR_SPEED);
      leftMotor.forward();
      rightMotor.forward();
      LCD.drawString("value: " + getIntensity(), 0, 5);

      if (getIntensity() < lineThreshold) {
        break;
      }
    }
    leftMotor.stop(true);
    rightMotor.stop();
    Sound.beep();


    leftMotor.setSpeed(MOTOR_SPEED);
    rightMotor.setSpeed(MOTOR_SPEED);
    leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_OFFSET), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_OFFSET), false);



    while (lineCounter < 4) {
      turnClockwise();


      if (getIntensity() < lineThreshold) {
        ThetaArray[lineCounter] = odometer.getXYT()[2];
        Sound.setVolume(10);
        Sound.systemSound(false, 4);
        lineCounter++;
      }
    }


    double dX, dY, anglex, angley;

    // Get our location from origin using the calculated angles
    angley = ThetaArray[3] - ThetaArray[1];
    anglex = ThetaArray[2] - ThetaArray[0];

    // we then compute the x and y coordinates based on anglex and angley

    dX = -SENSOR_OFFSET * Math.cos(Math.toRadians(angley / 2));
    dY = -SENSOR_OFFSET * Math.cos(Math.toRadians(anglex / 2));

    // we set the current position in odometer to the x and y coordinate we just calculated
    odometer.setXYT(dX, dY, odometer.getXYT()[2]);

    this.travelTo(0.0, 0.0);
    do {
      LCD.drawString("X: " + dX, 0, 5);
      LCD.drawString("Y: " + dY, 0, 6);
    } while (isNavigating());


    Sound.setVolume(20);
    Sound.systemSound(false, 2);

    // rotate the robot by the negative of its current heading to align it with the 0-degree axis
    double currentangle = odometer.getXYT()[2];
    turnTo(-currentangle);


  }

  /**
   * This method makes the robot turn clockwise.
   */
  private void turnClockwise() {
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * 
   * this methods reads the robot's current position from the odometer , and calculates how far it needs to move based
   * on that
   * 
   * Based on in which quadrant it needs to move, the angle is calculated differently to ensure minimal angle
   * 
   * This method was borrowed from the navigation class
   * 
   * @param x - x coordinate of the waypoint it has to travel to
   * @param y - y coordinate of the waypoint it has to travel to
   */
  void travelTo(double x, double y) {

    double currentX, currentY, currentT, deltaX, deltaY, dist;
    double dTheta = 0;
    double position[] = new double[3];
    position = odometer.getXYT();
    currentX = position[0];
    currentY = position[1];
    currentT = position[2];

    deltaX = x - currentX;
    deltaY = y - currentY;
    dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);



    if (deltaY >= 0) {


      dTheta = Math.atan(deltaX / deltaY);


    } else if (deltaY <= 0 && deltaX >= 0) {


      dTheta = Math.atan(deltaX / deltaY) + Math.PI;

    } else {


      dTheta = Math.atan(deltaX / deltaY) - Math.PI;
    }

    double thetaDiff = (Math.toDegrees(dTheta) - currentT);

    turnTo(thetaDiff);

    leftMotor.setSpeed(250);
    rightMotor.setSpeed(250);
    leftMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, dist), false);



  }

  /**
   * this method takes an angle and returns the minimum angle needed to turn to reach the next waypoint. ex: if
   * thetaDiff indicates that the robot should turn 240 degree in a certain direction. It is the same as turning 120
   * degree in the other direction. We therefore, minimize the angle
   * 
   * the method was borrowed from lab3 navigation class
   * 
   * @param theta
   */
  private void turnTo(double theta) {

    if (theta > 180) {


      theta = 360 - theta;
      leftMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);

    }

    else if (theta < -180) {


      theta = 360 + theta;
      leftMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);

    }

    else {


      leftMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);

    }
  }

  /**
   * this boolean check if the robot's motors are moving and therefore we know if the robot is navigating or not
   * 
   * @return
   */
  private boolean isNavigating() {

    boolean result = false;
    if (leftMotor.isMoving() && rightMotor.isMoving()) {
      result = true;
    }

    return result;
  }


  /**
   * This is a simple getter function that gets the value read by the light sensor and simply returns it .
   * 
   * @return a double value of the light sensor reading.
   */
  private double getIntensity() {
    centerLightSensor.fetchSample(lsData, 0);
    double lsValue = lsData[0] * 1000;
    return lsValue;

  }

  /**
   * this is a filter that we use for our light sensor, it takes in 5 successive values read by the light sensor and
   * filters them by taking their average
   * 
   * @return the average intensity of 5 measurements
   */

  private double lightFilter() {

    double[] values = new double[5];
    for (int i = 0; i < 5; i++) {
      values[i] = getIntensity();
    }

    double avg = 0;

    for (int k = 0; k < values.length; k++) {

      avg += values[k];

    }

    avg = avg / 5;

    return avg;

  }

  /**
   * convert distance and convert angles were both methods taken from the the SquareDriver class they convert distance
   * or angle respectively into the appropriate number of tacho counts the motor has to perform.
   * 
   * 
   * @param radius
   * @param distance
   * @return
   */

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * convert distance and convert angles were both methods taken from the the SquareDriver class they convert distance
   * or angle respectively into the appropriate number of tacho counts the motor has to perform.
   * 
   * 
   * @param radius
   * @param distance
   * @return
   */

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
