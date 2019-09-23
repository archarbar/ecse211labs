package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private float[] csData;
  private SampleProvider colorSampleProvider= colorSensor.getRedMode();
  private static final float LINE_RED_INTENSITY = 0.3f;
  
  public OdometryCorrection() {
    this.odometer = Odometer.getOdometer();
    csData = new float[colorSensor.sampleSize()];
  }
  /*
   * Here is where the odometer correction code should be run.
   */
  public void run() {
    long correctionStart, correctionEnd;
    int tileCountX = 0, tileCountY = 0;
    //set initial odometer position to 0
    odometer.setX(0);
    odometer.setY(0);

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      colorSampleProvider.fetchSample(csData, 0); //retrieve data from sensor


      // TODO Calculate new (accurate) robot position
      if(csData[0] < LINE_RED_INTENSITY) {
        Sound.beep();
        double currentPosition[] = odometer.getXYT();
        double x = currentPosition[0];
        double y = currentPosition[1];
        double theta = currentPosition[2];
        
        // if going north or east, increase tile count before calculation.
        // if going south or west, decrease tile count after calculation.
        if (north(theta)) { //if going north (increasing y direction)
          tileCountY++;
          y = TILE_SIZE*tileCountY;
        } else if (south(theta)) { //if going south (decreasing y direction)
          y = TILE_SIZE*tileCountY;
          tileCountY--;
        } else if (east(theta)) { //if going east (increasing x direction)
          tileCountX++;
          x = TILE_SIZE*tileCountX;
        } else if (west(theta)) { //if going west (decreasing x direction)
          x = TILE_SIZE*tileCountX;
          tileCountX--;
        }

        // update the odometer values with data from the light sensor
        odometer.setXYT(x,y,theta);
      }
      // this ensures the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }

  private boolean north(double theta) { // verify angle to check if robot going north
    return (theta<45 || theta>315);
  }
  private boolean south(double theta) { // verify angle to check if robot going south
    return (theta<225 && theta>135);
  }
  private boolean west(double theta) { // verify angle to check if robot going west
    return (theta<315 && theta>225);
  }
  private boolean east(double theta) { // verify angle to check if robot going east
    return (theta<135 && theta>45); 
  }


}