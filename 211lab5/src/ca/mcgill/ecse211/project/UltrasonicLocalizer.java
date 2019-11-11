package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import static ca.mcgill.ecse211.lab5.Resources.*;
import java.util.Arrays;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;



public class UltrasonicLocalizer {


  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;


  private float[] usData;
  private SampleProvider usSamples;


  private static final int ROTATE_SPEED = 50;
  private double deltaT = 0;

  private static final int WALL_DISTANCE = 22;
  private int errorMargin = 3;// error margin to counter the effect of noise
  private double TURNING_ERROR = 3.5;

  /**
   * Current location within the filter array.
   */
  int filterControl = 0;

  /**
   * Size of the median filter buffer.
   */
  int filterSize = 10;
  /**
   * Array of previous detected distances.
   */
  int[] sample = new int[filterSize];
  /**
   * A sorted array populated with the previous samples.
   */
  int[] sortedSamples = new int[filterSize];


  private Odometer odometer;

  // enum used to distinguish between the two methods.

  public enum edgeType {
    FallingEdge, RisingEdge;
  }

  private edgeType type;// edgeType initializer

  /**
   * This is the constructor of the class. It sets up both motors, the odometer, the track between the wheels, the
   * radius of the wheels, the edgetype and the ultrasonic sensor
   * 
   * @param leftMotor
   * @param rightMotor
   * @param odometer
   * @param TRACK
   * @param WHEEL_RAD
   * @param type
   * @param usSamples
   * @throws OdometerExceptions
   */
  public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
      final double TRACK, final double WHEEL_RAD, edgeType type, SampleProvider usSamples) {


    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;


    this.odometer = odometer;


    this.type = type;


    this.usSamples = usSamples;
    this.usData = new float[usSamples.sampleSize()];


  }

  /**
   * This is the main method. It calls the fallingEdge() and risingEdge() methods when needed
   */
  public void mainMethod() {
    // The main threaded method.
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    fillFilter();

    if (type == edgeType.FallingEdge) {
      fallingEdge();
    } else if (type == edgeType.RisingEdge) {
      risingEdge();
    } else {
      throw new IllegalArgumentException();
    }

  }

  /**
   * this methods detects wall when the distance falls below a treshold(WALL_DISTANCE). It starts by making the robot
   * move away from the wall to make sure it doesn't face it. Then, the robot starts rotating in the same direction
   * until it detects a first wall. the theta value is then recorded and the robot starts rotating in the opposite
   * direction until the distance read falls again below the threshold ( = second wall is detected) and also records the
   * value of theta. Once we have recorded both theta values we can now make the robot orient itself to the 0-degree
   * axis.
   * 
   */
  public void fallingEdge() {


    while (getDistance() < WALL_DISTANCE + errorMargin) {
      turnCounterclockwise();
    }


    while (getDistance() > WALL_DISTANCE) {
      turnCounterclockwise();

    }
    Sound.beep();
    double firstAngle = odometer.getXYT()[2];


    while (getDistance() < WALL_DISTANCE + errorMargin) {
      turnClockwise();
    }


    while (getDistance() > WALL_DISTANCE) {
      turnClockwise();
    }
    Sound.beep();
    double secondAngle = odometer.getXYT()[2];

    stop();

    // we implement 2 cases depending on which angle is bigger than the other one and store the result in a deltaT
    // variable
    // this variable will be used below to calculate the theta value by which the robot has to turn in order to be
    // aligned with the 0-degree axis

    if (firstAngle > secondAngle) {
      deltaT = 225 - (firstAngle + secondAngle) / 2;
    } else {
      deltaT = 45 - (firstAngle + secondAngle) / 2;
    }


    double turnAngle = deltaT + odometer.getXYT()[2];

    // rotate to 0 degree axis
    leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turnAngle - TURNING_ERROR), true);
    rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turnAngle - TURNING_ERROR), false);

    odometer.setXYT(0.0, 0.0, 0.0);

  }



  /**
   * Contrary to the fallingEdge method this method detects a wall when the distance goes pass a
   * trseshold(WALL_DISTANCE) but works in the exact same manner: it will start by making sure the robot is facing the
   * wall, it will then turn until it the distance reported by the US sensor is greater than the specified threshold,
   * and record the theta value. then it will rotate in the opposite direction until the same event happens and record
   * the second theta value. Once we have recorded both theta values we can now make the robot orient itself to the
   * 0-degree axis
   */
  public void risingEdge() {


    while (getDistance() > WALL_DISTANCE) {
      turnCounterclockwise();
    }


    while (getDistance() < WALL_DISTANCE + errorMargin) {
      turnCounterclockwise();
    }
    Sound.beep();
    double firstAngle = odometer.getXYT()[2];


    while (getDistance() > WALL_DISTANCE) {
      turnClockwise();
    }


    while (getDistance() < WALL_DISTANCE + errorMargin) {
      turnClockwise();
    }
    Sound.beep();
    double secondAngle = odometer.getXYT()[2];

    stop();

    // we implement 2 cases depending on which angle is bigger than the other one and store the result in a deltaT
    // variable
    // this variable will be used below to calculate the theta value by which the robot has to turn in order to be
    // aligned with the 0-degree axis
    if (firstAngle > secondAngle) {
      deltaT = 225 - (firstAngle + secondAngle) / 2 + 180;
    } else {
      deltaT = 45 - (firstAngle + secondAngle) / 2 + 180;
    }

    double turnAngle = deltaT + odometer.getXYT()[2];

    // rotate to 0 degree axis
    leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turnAngle - TURNING_ERROR), true);
    rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turnAngle - TURNING_ERROR), false);


    odometer.setXYT(0.0, 0.0, 0.0);
  }

  /**
   * this is a simple getter method that gets the distance from the ultrasonic sensor
   * 
   * @return
   */
  public int getDistance() {
    usSamples.fetchSample(usData, 0);
    int distance = (int) (usData[0] * 100);
    return medianFilter(distance);

  }

  /**
   * convert distance and convert angles were both methods taken from the the SquareDriver class they convert distance
   * or angle respectively into the appropriate number of tacho counts the motors have to perform.
   * 
   * 
   * @param radius
   * @param distance
   * @return tacho counts
   */


  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * convert distance and convert angles were both methods taken from the the SquareDriver class they convert distance
   * or angle respectively into the appropriate number of tacho counts the motors have to perform.
   * 
   * 
   * @param radius
   * @param distance
   * @return tacho counts
   */

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * makes the robot turn clockwise
   */
  public void turnClockwise() {
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * makes the robot turn counterclockwise
   */
  public void turnCounterclockwise() {
    leftMotor.backward();
    rightMotor.forward();
  }

  /**
   * makes the robot stop
   */
  public void stop() {
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * Fills the filter with the current distance, so not skew the initial values.
   */
  private void fillFilter() {
    usSamples.fetchSample(usData, 0); // acquire distance data in meters
    int distance = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
    for (int i = 0; i < sample.length; i++) {
      sample[i] = distance;
    }
  }

  /**
   * A median filter for the ultrasonic sensor.
   * 
   * @param distance
   */
  private int medianFilter(int distance) {
    sample[filterControl] = distance;
    filterControl++;
    filterControl %= sample.length;

    sortedSamples = Arrays.copyOf(sample, sample.length);
    Arrays.sort(sortedSamples);
    int median = sortedSamples[sortedSamples.length / 2];
    if (distance > median) {
      distance = median;
    }
    return distance;
  }

}
