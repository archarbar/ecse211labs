package ca.mcgill.ecse211.lab4;

//import static ca.mcgill.ecse211.lab3.Resources.US_SENSOR;
//import static ca.mcgill.ecse211.lab4.Resources.FORWARD_SPEED;
//import static ca.mcgill.ecse211.lab4.Resources.leftMotor;
//import static ca.mcgill.ecse211.lab4.Resources.rightMotor;
//import static ca.mcgill.ecse211.lab4.Resources.LCD;
import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer{
  private float[] usData;
  private int[] us_dists;
  private int dist;
  private int booler=0;
//  private Odometer odometer;
  int distance;
  int filterControl;
  private SampleProvider usSampler= US_SENSOR.getDistanceMode();
  
 
  
  public UltrasonicLocalizer(){
//    this.odometer = Odometer.getOdometer();
    usData = new float[US_SENSOR.sampleSize()];
  }
  
  public void run() {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire distance data in meters
    dist = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
    filter(dist);
    LCD.drawString("USDIST:  "+String.valueOf(this.distance), 0, 4);

    if(this.distance<100) {
//      Sound.beep();
      leftMotor.forward();
      rightMotor.backward(); //actually going backwards
      while(booler!=2) {
        US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire distance data in meters
        dist = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
        filter(dist);
        LCD.drawString("USDIST:  "+String.valueOf(this.distance), 0, 4);
        if((booler==0)&&(this.distance>255)) {
          booler=1;
        }
        if((booler==1)&&(this.distance<30)) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          booler=2;
        }
        try {
          Thread.sleep(10);
        } catch (Exception e) {
        } // Poor man's timed sampling
        
      }
      US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire distance data in meters
      dist = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
      filter(dist);
      LCD.drawString("USDIST:  "+String.valueOf(this.distance), 0, 5);
      
    }
    else {
      Sound.beep();
    }
  }


  private void fallingEdge() {
    
  }
  
  private void risingEdge() {

  }
  
  int getDist() {
    usSampler.fetchSample(usData, 0); // acquire distance data in meters
    dist = (int) (usData[0] * 100.0);
    filter(dist);
    return this.distance;
  }

  int getCleanDist() {
    int sumDist=0;
    
    for(int i=0; i<10; i++) {
//      usSampler.fetchSample(usData, 0); // acquire distance data in meters
//      dist = (int) (usData[0] * 100.0);
//      filter(dist);
      sumDist+=getDist();
      try {
        Thread.sleep(10);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
    return sumDist/10;
    
  }
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
