// Lab4.java
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;

// static import to avoid duplicating variables and make the code easier to read


/**
 * The main driver class for the localization lab.
 */
public class Main {
  public static int buttonChoice;
  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {
    buttonChoice = chooseRisingOrFalling();
    new Thread(new UltrasonicLocalizer()).start();
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
  private static int chooseRisingOrFalling() {
    int buttonChoice;
    Display.showText("< Left | Right >",
                     "       |        ",
                     " Rising|Falling ",
                     "  edge | edge   ",
                     "       |        ");
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
