// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intakePneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFloat extends CommandBase{//extends InstantCommand {
  intakePneumaticSubsystem intakePneumatic;
  boolean m_bForward;

  public IntakeFloat(boolean bForward) {

    m_bForward = bForward;

    intakePneumatic = intakePneumaticSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    if(intakePneumatic != null)
      addRequirements(intakePneumatic);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ( intakePneumatic != null )
    {
      if(m_bForward == true)
       intakePneumatic.setFloatForward();
      else
       intakePneumatic.setFloatReverse();
    }
    
  }

  @Override
  public void execute()
  {

  }
  
  @Override
  public void end(boolean interrupted)
  {
    //don't stopFloat()
    // if ( intakePneumatic != null)
    // intakePneumatic.stopFloat();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
