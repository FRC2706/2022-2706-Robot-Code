// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.SwitchSubsystem;

public class TestSwitch extends CommandBase {
  private SwitchSubsystem switchDetector;

  /** Creates a new TestSwitch. */
  public TestSwitch() {
    switchDetector = new SwitchSubsystem(Config.INDEXER_SWITCH);
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(switchDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( switchDetector.getResult() == true )
    {
      System.out.println("Switch detected");
    }
    System.out.println("Switch read" + switchDetector.getResult());

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
