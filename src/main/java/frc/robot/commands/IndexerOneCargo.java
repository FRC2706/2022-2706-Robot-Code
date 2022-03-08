// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.config.Config;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.SwitchSubsystem;

public class IndexerOneCargo extends CommandBase {

  private IndexerSubSystem indexer;
  private SwitchSubsystem switchDetector;
  private final int TARGET_RPM = 1000;

  /** Creates a new IndexerOneCargo. */
  public IndexerOneCargo() {

    indexer = IndexerSubSystem.getInstance();
    switchDetector = new SwitchSubsystem(7);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(switchDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      if(switchDetector.isDetected() == true)
      {
        indexer.stop();
      }
      else
      {
        indexer.setTargetRPM(TARGET_RPM);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
