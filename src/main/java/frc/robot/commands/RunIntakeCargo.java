// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeCargoSubsystem;

public class RunIntakeCargo extends CommandBase {
  IntakeCargoSubsystem intakeCargoSubsystem;

  private double timeout;
  private Timer timer;
  private boolean m_bUseTimer = false;
  private boolean m_bForward;

  /** Creates a new RunPneumaticsIntake. */
  public RunIntakeCargo( boolean bForward, int timeOut ) {
    timeout = timeOut;
    m_bForward = bForward;

    if ( timeout > 0 )
    {
      m_bUseTimer = true;
      timer = new Timer();
    }
    else
    {
      m_bUseTimer = false;
    }

    intakeCargoSubsystem = IntakeCargoSubsystem.getInstance();
    if (intakeCargoSubsystem != null)
    {
      addRequirements(intakeCargoSubsystem);
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if( m_bUseTimer == true )
    { 
        timer.start();
        timer.reset();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeCargoSubsystem != null)
    {
      if (m_bForward == true)
        intakeCargoSubsystem.start();
      else
       intakeCargoSubsystem.startReverse();;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (intakeCargoSubsystem != null)
    {
      intakeCargoSubsystem.stop();
    }

    if( m_bUseTimer == true )
     timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( m_bUseTimer == true )
    return timer.get() > timeout;
else
    return false;

  }
}
