/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorSubsystem extends SubsystemBase {
  private final int AVERAGE_NUMBER = 10;
  private final int RED_THRESHOLD = 500;
  private final int RED_PROXIMITY_THRESHOLD = 275;
  private final int BLUE_THRESHOLD = 450;
  private final int BLUE_PROXIMITY_THRESHOLD = 275;
  
  private NetworkTableEntry redValue, blueValue, irValue, proximityValue; 
  private NetworkTableEntry detectRed, detectBlue;
  private LinearFilter filterRed, filterBlue, filterIR, filterProximity;

  private boolean bDetectedCargo = false;
  private boolean bColorSensorGood = false;
  private final ColorSensorV3 colorSensor;
  private static final ColorSensorSubsystem INSTANCE_COLOR_SENSOR_SUBSYSTEM = new ColorSensorSubsystem();

  public ColorSensorSubsystem() {
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    if(colorSensor != null)
    {
      bColorSensorGood = true;
    }
  
    var table = NetworkTableInstance.getDefault().getTable("colorSensor");
    redValue = table.getEntry("redValue");
    blueValue = table.getEntry("blueValue");
    irValue = table.getEntry("IRValue");
    proximityValue = table.getEntry("proximityValue");
    detectRed = table.getEntry("detectRed");
    detectBlue = table.getEntry("detectBlue");
    filterRed = LinearFilter.movingAverage(AVERAGE_NUMBER);
    filterBlue = LinearFilter.movingAverage(AVERAGE_NUMBER);
    filterIR = LinearFilter.movingAverage(AVERAGE_NUMBER);
    filterProximity = LinearFilter.movingAverage(AVERAGE_NUMBER);

  
  }
  public static ColorSensorSubsystem getInstance() {
    if ( INSTANCE_COLOR_SENSOR_SUBSYSTEM.bColorSensorGood == true)
      return INSTANCE_COLOR_SENSOR_SUBSYSTEM;
    else{
      return null;
    }
  }

  /**
   * Returns the red value of the sensed color
   * 
   * @return Red value
   */
  public int getRed() {
    return colorSensor.getRed();
  }

  /**
   * Returns the blue value of the sensed color
   * 
   * @return Blue value
   */
  public int getBlue() {
    return colorSensor.getBlue();
  }

  /**
   * Returns the green value of the sensed color
   * 
   * @return Green value
   */
  public int getGreen() {
    return colorSensor.getGreen();
  }

  /**
   * Returns the IR value of the sensor
   * 
   * @return IR value
   */
  public int getIR() {
    return colorSensor.getIR();
  }

  /**
   * Returns the sensed proximoty of the sensor
   * 
   * @return Proximity value
   */
  public int getProximity() {
    return colorSensor.getProximity();
  }

  /**
   * Returns a Color object representing the sensed color
   * 
   * @return Color value
   */
  public Color getColor() {
    return colorSensor.getColor();
  }

  /**
   * Returns a raw color value
   * 
   * @return Raw color
   */
  public RawColor getRawColor() {
    return colorSensor.getRawColor();
  }

  // public boolean isDetected() {
  //   return bDetectedCargo;
  // }

  public boolean isDetected() {

    //proximity sensor range: 1cm to 10cm

    /* Used instance variables
    int proximity = colorSensor.getProximity();
    int ir = colorSensor.getIR();
    int red = colorSensor.getRed();
    int blue = colorSensor.getBlue();
    */

    double red = filterRed.calculate(colorSensor.getRed());
    double blue = filterBlue.calculate(colorSensor.getBlue());
    double ir = filterIR.calculate(colorSensor.getIR());
    double proximity = filterProximity.calculate(colorSensor.getProximity());

    boolean isRed = false;
    boolean isBlue = false;

    //First, detect the red cargo. If it is not detected, then detect the blue cargo
    if(red > RED_THRESHOLD && proximity > RED_PROXIMITY_THRESHOLD)
    {
      isRed = true;
    }
    else if(blue > BLUE_THRESHOLD && proximity > BLUE_PROXIMITY_THRESHOLD)
    {
      isBlue = true;
    }

    redValue.setNumber((double)red);
    blueValue.setNumber((double)blue);
    irValue.setNumber((double)ir);
    proximityValue.setNumber((double) proximity);
    detectRed.setBoolean(isRed);
    detectBlue.setBoolean(isBlue);
   
    bDetectedCargo = isRed || isBlue;
    return bDetectedCargo;
  }
}