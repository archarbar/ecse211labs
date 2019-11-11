package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

  private Odometer odo;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd) {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
  }

  /**
   * Overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd, long timeout) {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  /**
   * retrieve the X,Y,T coordinates from the odometer and display them, also makes sure the data is update only once a
   * period
   */
  public void run() {

    lcd.clear();

    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();


      position = odo.getXYT();


      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);

      // ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

  /**
   * Shows the text on the LCD, line by line.
   *
   * @param strings comma-separated list of strings, one per line
   */
  public static void showText(String... strings) {
    Resources.LCD.clear();
    for (int i = 0; i < strings.length; i++) {
      Resources.LCD.drawString(strings[i], 0, i);
    }
  }

}
