package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private float[] csData;
  private SampleProvider colorSampleProvider= colorSensor.getRedMode();
  
  public OdometryCorrection() {
    this.odometer = Odometer.getOdometer();
    csData = new float[colorSensor.sampleSize()];
  }
  /*
   * Here is where the odometer correction code should be run.
   */
  public void run() {
    long correctionStart, correctionEnd;
    float lastSensorData;
    int tileCountX = 0, tileCountY = 0;
    //set initial odometer position to center of tile
    odometer.setX(TILE_SIZE/2);
    odometer.setY(TILE_SIZE/2);

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      colorSampleProvider.fetchSample(csData, 0); //retrieve data from sensor
      csData[0] *= 1000; //scale sensor output to make it easier to read and more accurate


      boolean correcting = false;




      // TODO Calculate new (accurate) robot position
      if (correcting) {
        double currentPosition[] = odometer.getXYT();
        double x = currentPosition[0];
        double y = currentPosition[1];
        double currentTheta = currentPosition[2];
        
        // if increasing the direction, update the counter first, if decreasing the direction, update the counter after calculating current location
        if (north(currentTheta)) { //increasing Y direction
          tileCountY++;
          y = TILE_SIZE*tileCountY;
        } else if (east(currentTheta)) { //increasing X direction
          tileCountX++;
          x = TILE_SIZE*tileCountX;
        } else if (south(currentTheta)) { //decreasing Y direction
          y = TILE_SIZE*tileCountY;
          tileCountY--;
        } else if (west(currentTheta)) { //decreasing X direction
          x = TILE_SIZE*tileCountX;
          tileCountX--;
        }

        // Updates the odometer with new calculated (and more accurate) values.
        odometer.setXYT(x,y,currentTheta);
      }
      // this ensures the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }

  private boolean north(double theta) {
    return (theta<45 || theta>315);
  }
  private boolean south(double theta) {
    int direction = 180;
    return (theta<direction+45 && theta>direction-45);
  }
  private boolean west(double theta) {
    int direction = 270;
    return (theta<direction+45 && theta>direction-45);
  }
  private boolean east(double theta) {
    int direction = 90;
    return (theta<direction+45 && theta>direction-45);
  }


}