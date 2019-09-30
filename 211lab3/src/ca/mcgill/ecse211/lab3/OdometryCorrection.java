package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private float[] csData;
  private SampleProvider colorSampleProvider= colorSensor.getRedMode(); // use a red light to compare luminence level
  private static final float LINE_RED_INTENSITY = (float) 0.3; // cast to float since default is double
  
  public OdometryCorrection() {
    this.odometer = Odometer.getOdometer();
    csData = new float[colorSensor.sampleSize()];
  }
  /*
   * Here is where the odometer correction code should be run.
   */
  public void run() {
    long correctionStart, correctionEnd;
    
    //set initial odometer position to 0
    odometer.setX(0);
    odometer.setY(0);

    int XtileCount = 0, YtileCount = 0;
    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position
      colorSampleProvider.fetchSample(csData, 0); //get data from sensor
      if(csData[0] < LINE_RED_INTENSITY) { //if light read by sensor is smaller (darker) than red light, eg., black lines
        Sound.beep(); // sound alert to know when the sensor hits a line
        double currentPosition[] = odometer.getXYT();
        double x = currentPosition[0]; //get current x position
        double y = currentPosition[1]; //get current y position
        double theta = currentPosition[2]; // get current theta angle
        
        // if going north or east, increase tile count before calculation.
        // if going south or west, decrease tile count after calculation.
        if (north(theta)) { //if going north (increasing y direction)
          YtileCount++;
          y = TILE_SIZE*YtileCount;
        } else if (south(theta)) { //if going south (decreasing y direction)
          y = TILE_SIZE*YtileCount;
          YtileCount--;
        } else if (east(theta)) { //if going east (increasing x direction)
          XtileCount++;
          x = TILE_SIZE*XtileCount;
        } else if (west(theta)) { //if going west (decreasing x direction)
          x = TILE_SIZE*XtileCount;
          XtileCount--;
        }

        //TODO Update odometer with new calculated (and more accurate) values
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

  boolean north(double theta) { // verify angle to check if robot going north
    return (theta<45 || theta>315);
  }
  boolean south(double theta) { // verify angle to check if robot going south
    return (theta<225 && theta>135);
  }
  boolean west(double theta) { // verify angle to check if robot going west
    return (theta<315 && theta>225);
  }
  boolean east(double theta) { // verify angle to check if robot going east
    return (theta<135 && theta>45); 
  }


}