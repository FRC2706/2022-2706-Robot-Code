// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AnalogSensor;

public class ReadAnalogInput extends CommandBase {
  AnalogSensor analogInput = null;
  int m_analogPort;

  private NetworkTableEntry portNum, V, AvgV;

  /** Creates a new ReadAnalogInput. */
  public ReadAnalogInput( int analogPort) {
    m_analogPort = analogPort;

    // Use addRequirements() here to declare subsystem dependencies.
    if ( analogPort != -1)
    {
      analogInput = new AnalogSensor( analogPort );
      addRequirements(analogInput);
    }
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var table = NetworkTableInstance.getDefault().getTable("analogInput");

    portNum = table.getEntry("port");
    V = table.getEntry("voltage");
    AvgV = table.getEntry("Avg Voltage");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( analogInput != null )
    {
      double analogV = analogInput.getVoltage();
      double analogAvg = analogInput.getAverageVoltage();

      portNum.setNumber((double)m_analogPort);
      V.setNumber(analogV);
      AvgV.setNumber(analogAvg);
    }
    
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
