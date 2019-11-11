package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.lab5.Resources.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.lab5.Display;
import ca.mcgill.ecse211.lab5.LightLocalizer;
import ca.mcgill.ecse211.lab5.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;

public class Main {
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  // private static final Port sensorPortUS = LocalEV3.get().getPort("S4"); // port will likely change


  /**
   * The main method initializes all the threads and depending on user input, it runs either Falling edge or Rising edge
   * method.
   * 
   * @param args
   */
  public static void main(String[] args) {

    @SuppressWarnings("resource")
    // SensorModes usSensor = new EV3UltrasonicSensor
    SampleProvider usDistance = US_SENSOR.getMode("Distance");



    int buttonChoice;

    Odometer odometer = Odometer.getOdometer();
    Display odometryDisplay = new Display(lcd);

    // LauncherControl.reset();



    do {
      // clear the displays
      lcd.clear();

      lcd.drawString("Stationa-| Mobile  >", 0, 0);
      lcd.drawString("ry Launch| Launch   ", 0, 1);
      lcd.drawString("         |          ", 0, 2);
      lcd.drawString("    <    |     >    ", 0, 3);
      lcd.drawString("         |          ", 0, 4);
      buttonChoice = Button.waitForAnyPress();
    }

    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_RIGHT) {
      int[] launchTarget = chooseTargetLocation();
      // double[] destination = Navigation.findLaunchLocation(launchTarget);
      double[][] targetTile = new double[1][2];
      double[] destination = Navigation.waypointToLocation(launchTarget);
      targetTile[0][0] = destination[0];
      targetTile[0][1] = destination[1];

      LightLocalizer lsLocalizer = new LightLocalizer(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD);
      Navigation navigation = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD, targetTile);


      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD,
          UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
      initThreads(odometer, odometryDisplay);
      // initiates the rising or falling edge localization .
      usLocalizer.mainMethod();


      // while (Button.waitForAnyPress() != Button.ID_RIGHT);


      // start the light localization
      lsLocalizer.mainMethod();



      // call navigation

      navigation.run();
      Sound.beep();


      // press on right button to
      // LauncherControl.reset();
      // if(Button.waitForAnyPress() == Button.ID_RIGHT) {
      // int launchSpeed = LauncherControl.calculateSpeed(120);
      // LauncherControl.launch(Resources.LAUNCH_SPEED);
      // Button.waitForAnyPress();
      // System.exit(0);
      // }
      // Button.waitForAnyPress();

    }


    // else if(buttonChoice == Button.ID_LEFT) {
    //
    // }
    // launch
    LauncherControl.reset();
    for (int i = 0; i < 5; i++) {
      waitForPress();
      LauncherControl.launch(LAUNCH_SPEED);

    }
    System.exit(0);


  }


  // private static int chooseSetup() {
  // int buttonChoice;
  // Display.showText("< Left | Right >", " | ", "Mobile |Station-", " | ary ", " launch| launch ");
  //
  // do {
  // buttonChoice = Button.waitForAnyPress(); // left or right press
  // } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
  // return buttonChoice;
  // }

  /*
   * this method lets the user input the target location directly from the robot screen and using robot buttons
   */
  private static int[] chooseTargetLocation() {

    DecimalFormat numberFormat = new DecimalFormat("0");
    int index = 0;
    int pos[] = {0, 0, 0, 0};
    String spaces = " ";
    lcd.clear();
    lcd.drawString("(" + numberFormat.format(pos[0]) + numberFormat.format(pos[1]) + "," + numberFormat.format(pos[2])
        + numberFormat.format(pos[3]) + ")", 0, 0);
    lcd.drawString(spaces + "^", 0, 1);
    int buttonPress = Button.waitForAnyPress();
    while (buttonPress != Button.ID_ENTER) {
      if (buttonPress == Button.ID_ESCAPE) {
        System.exit(0);
      } else if (buttonPress == Button.ID_LEFT) {
        index--;
        if (index < 0) {
          index += 4;
        }
        index %= 4;

      } else if (buttonPress == Button.ID_RIGHT) {
        index++;
        index %= 4;
      } else if (buttonPress == Button.ID_UP) {
        pos[index]++;
        pos[index] %= 10;
      } else if (buttonPress == Button.ID_DOWN) {
        pos[index]--;
        if (pos[index] < 0) {
          pos[index] += 10;
        }
        pos[index] %= 10;
      }

      spaces = "";
      for (int i = 0; i <= index; i++) {
        spaces += " ";
      }
      if (index > 1) {
        spaces += " ";
      }
      lcd.clear();
      lcd.drawString("(" + numberFormat.format(pos[0]) + numberFormat.format(pos[1]) + "," + numberFormat.format(pos[2])
          + numberFormat.format(pos[3]) + ")", 0, 0);
      lcd.drawString(spaces + "^", 0, 1);



      buttonPress = Button.waitForAnyPress();
    }

    lcd.clear();
    int positions[] = new int[2];
    positions[0] = 10 * pos[0] + pos[1];
    positions[1] = 10 * pos[2] + pos[3];
    return positions;
  }

  /**
   * this method simply initiates the threads for the odometer and odometer display
   * 
   * @param odometer
   * @param odometryDisplay
   */
  private static void initThreads(Odometer odometer, Display odometryDisplay) {
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();
  }


  public static void waitForPress() {
    Display.showText("Please press any ", "button to start  ", "the launcher    ", "                ",
        "                ");
    if (Button.waitForAnyPress() == Button.ID_ESCAPE) {
      System.exit(0);
    }
    try {
      Thread.sleep(5000);
    } catch (Exception e) {

    }
  }



}
