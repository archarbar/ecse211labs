package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import java.util.concurrent.locks.Condition;

public class Sweeper extends Thread{
  
  private static Sweeper swep;


  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   * 
   * @return the Odometer Object
   */
  public synchronized static Sweeper getSweeper() {
    if (swep == null) {
      swep = new Sweeper();
    }
    
    return swep;
  }
//  private Sweeper() {
//    setPause(false);
//  }
//  
//  public void setPause(boolean pause) {
//    this.pauseSweep=pause;
//  }
//  @Override
//  public int readUSDistance() {
//      return this.distanceUS;
//  }

//
//  @Override
//  public void processUSData(int distance) {
//      if (distance >= 255 && filterControl < FILTER_OUT) {
//          // bad value, do not set the distance var, however do increment the
//          // filter value
//          filterControl++;
//      } else if (distance >= 255) {
//          // We have repeated large values, so there must actually be nothing
//          // there: leave the distance alone
//          this.distanceUS = distance;
//      } else {
//          // distance went below 255: reset filter and leave
//          // distance alone.
//          filterControl = 0;
//          this.distanceUS = distance;
//      }
  public boolean pauseSweep=false;
  
  public void run(){
    sensorMotor.stop();
    sensorMotor.setAcceleration(3000);
    sensorMotor.setSpeed(100);
    while(true) {
    sensorMotor.rotate(90, false);
    sensorMotor.rotate(-90, false);
//    if(pauseSweep=true) {
//      break
    }
  }
}
