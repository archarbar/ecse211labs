package ca.mcgill.ecse211.lab1;

import lejos.hardware.Button;

/** HelloWorld example: prints to screen and waits for button press.

 */

public class HelloWorld {
  
  public static void main(String[] args) {
    System.out.println("Hello World!");
    Button.waitForAnyPress();
  }
}