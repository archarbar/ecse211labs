package ca.mcgill.ecse211.lab4;

import java.text.DecimalFormat;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab4.Resources.*;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  public void run() {
    
    LCD.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();


      
      // this ensures that the data is updated only once every period
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
   * Sets the timeout in ms.
   * 
   * @param timeout
   */
  public void setTimeout(long timeout) {
    this.timeout = timeout;
  }
  
  /**
   * Shows the text on the LCD, line by line.
   * 
   * @param strings comma-separated list of strings, one per line
   */
  public static void showText(String... strings) {
    LCD.clear();
    for (int i = 0; i < strings.length; i++) {
      LCD.drawString(strings[i], 0, i);
    }
  }

}
