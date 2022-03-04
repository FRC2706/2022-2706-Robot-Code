// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorSubsystem;

public class ReadColorSensor extends CommandBase {

  private final int AVERAGE_NUMBER = 10;
  private final int RED_THRESHOLD = 130;
  private final int RED_PROXIMITY_THRESHOLD = 60;
  private final int BLUE_THRESHOLD = 110;
  private final int BLUE_PROXIMITY_THRESHOLD = 55;

  ColorSensorSubsystem colorSensor;
  private NetworkTableEntry redValue, blueValue, irValue, proximityValue; 
  private NetworkTableEntry detectRed, detectBlue;
  private LinearFilter filterRed, filterBlue, filterIR, filterProximity;

  /** Creates a new ReadColorSensor. */
  public ReadColorSensor() {

    colorSensor = new ColorSensorSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
