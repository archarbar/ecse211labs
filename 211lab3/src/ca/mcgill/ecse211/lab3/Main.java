// Lab3.java
package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;

// static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;
import ca.mcgill.ecse211.lab3.UltrasonicController;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;


/**
 * The main driver class for the odometry lab.
 */
public class Main {

  public static UltrasonicController selectedController;
  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {
    int buttonChoice;
    new Thread(odometer).start(); // TODO implement Odometer
    
    buttonChoice = chooseAvoidanceOrNot();

    if (buttonChoice == Button.ID_LEFT) {
      new Thread(new Navigation()).start();
    }
    else {
      new Thread(sweeper).start();
      selectedController = new NavigationCorrection();
      new Thread(new NavigationCorrection()).start();
      new Thread(new UltrasonicPoller()).start();
    }
    
    new Thread(new Display()).start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing
    
    System.exit(0);
  }

  /**
   * Asks the user whether the motors should drive in a square or float.
   * 
   * @return the user choice
   */
  private static int chooseAvoidanceOrNot() {
    int buttonChoice;
    Display.showText("< Left | Right >",
                     "       |        ",
                     " No    | With   ",
                     "avoidan| avoidan",
                     " -ce   | -ce    ");
    
    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }
  
  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }
  
}
