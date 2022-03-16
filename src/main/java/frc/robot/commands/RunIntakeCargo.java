// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeCargoSubsystem;

public class RunIntakeCargo extends CommandBase {
  IntakeCargoSubsystem intakeCargoSubsystem;
  /** Creates a new RunPneumaticsIntake. */
  public RunIntakeCargo() {
    intakeCargoSubsystem = IntakeCargoSubsystem.getInstance();
    addRequirements(intakeCargoSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeCargoSubsystem.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeCargoSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
