package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

public class LauncherControl {

  /*
   * Launch ball with lauch acceleration and speeds. After launch, call reset motor method for next launch
   */
  public static void launch(int speed) {
    launcher.setAcceleration(LAUNCHER_ACCELERATION);
    launcher.setSpeed(speed);
    launcher.rotate(LAUNCHER_ANGLE, false);
    try {
      Thread.sleep(3000);
    } catch (Exception e) {
      // do nothing
    }
    reset();
  }

  /*
   * Reset motor to launch position for next launch
   */
  public static void reset() {
    launcher.setSpeed(RESET_SPEED);
    launcher.rotate(-LAUNCHER_ANGLE, false);
    launcher.stop();
  }


  public static int calculateSpeed() {
    int speed;
    double angleRad = Math.toRadians(LAUNCH_ANGLE);
    speed = (int) Math.sqrt((Math.pow((TARGET_DISTANCE * G * 100 / Math.cos(angleRad)), 2)
        / (2 * G * 100 * (LAUNCH_HEIGHT + TARGET_DISTANCE * Math.tan(angleRad)))));
    return speed;
  }

  public static int calculateSpeed(double distance) {
    int speed = 0;
    double angleRad = Math.toRadians(LAUNCH_ANGLE); // launch angle in radians
    double dSin = distance * Math.sin(2 * angleRad); // d sin(2*theta)
    double sin = Math.sin(angleRad); // sin(theta)
    double dividend = dSin * Math.sqrt(2 * G);
    double divisor = Math.sqrt(2 * dSin + LAUNCH_HEIGHT / (sin * sin));
    double quotient = dividend / divisor;
    speed = calculateMotorSpeed(quotient);
    return speed;
  }

  public static int calculateMotorSpeed(double tangentialV) {
    double angularV = tangentialV / LAUNCH_ARM_LENGTH;
    return (int) angularV;
  }
}
