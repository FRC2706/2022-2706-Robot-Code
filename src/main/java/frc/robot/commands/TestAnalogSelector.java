// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AnalogSelectorSubsystem;

public class TestAnalogSelector extends CommandBase {

  AnalogSelectorSubsystem m_analogSelector;

  /** Creates a new TestAnalogSelector. */
  public TestAnalogSelector() {
    m_analogSelector = AnalogSelectorSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    if(m_analogSelector != null)
    {
      addRequirements(m_analogSelector);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  public void readIndex()
  {
    if(m_analogSelector != null)
    {
      int selectorIndex = m_analogSelector.getIndex();
      System.out.println("selectorIndex: "+selectorIndex);
      System.out.println("getAverageVoltage(): "+m_analogSelector.getAverageVoltage());
    }
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    readIndex();
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
