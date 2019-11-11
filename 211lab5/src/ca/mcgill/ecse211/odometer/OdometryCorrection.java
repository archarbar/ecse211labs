package ca.mcgill.ecse211.odometer;

import static ca.mcgill.ecse211.lab5.Resources.*;
import ca.mcgill.ecse211.lab5.LineNavigation;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection {

  private static Odometer odometer = Odometer.getOdometer();

  public void run() {
    long updateStart, updateEnd;



    while (LineNavigation.lineNavigating) {
      updateStart = System.currentTimeMillis();
      if (lineDetected(sideLightSensor)) {
        double angle = odometer.getXYT()[2];
        if ((angle >= 350 || angle <= 10) || ((angle >= 170 && angle <= 190))) {
          double xPos = odometer.getXYT()[0];
          xPos = Math.round(xPos / TILE_SIZE) * TILE_SIZE;
          odometer.setX(xPos);
        } else if ((angle >= 80 && angle <= 100) || (angle >= 260 && angle <= 280)) {
          double yPos = odometer.getXYT()[1];
          yPos = Math.round(yPos / TILE_SIZE) * TILE_SIZE;
          odometer.setY(yPos);
        }
      }
      boolean centerLineDetected = lineDetected(centerLightSensor);
      if (!LineNavigation.turning) {
        if (centerLineDetected) {
          double angle = odometer.getXYT()[2];
          if (angle >= 335 || angle <= 25) {
            odometer.setTheta(0);
            double yPos = odometer.getXYT()[1];
            yPos = Math.round(yPos / TILE_SIZE) * TILE_SIZE;
            odometer.setY(yPos);
          } else if (angle >= 65 && angle <= 115) {
            odometer.setTheta(90);
            double xPos = odometer.getXYT()[0];
            xPos = Math.round(xPos / TILE_SIZE) * TILE_SIZE;
            odometer.setX(xPos);
          } else if (angle >= 155 && angle <= 205) {
            odometer.setTheta(180);
            double yPos = odometer.getXYT()[1];
            yPos = Math.round(yPos / TILE_SIZE) * TILE_SIZE;
            odometer.setY(yPos);
          } else if (angle >= 245 && angle <= 295) {
            odometer.setTheta(270);
            double xPos = odometer.getXYT()[0];
            xPos = Math.round(xPos / TILE_SIZE) * TILE_SIZE;
            odometer.setX(xPos);
          }
        } else {
          LineNavigation.findLine();
        }
      }


      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }

  }

  public static boolean lineDetected(EV3ColorSensor lightSensor) {
    return true;
  }
}
