// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ConditionalSubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeWithTime extends CommandBase {

  private IntakeSubsystem intakeSubsystem;
  private ConditionalSubsystemBase.SubsystemCondition condition;

 // private Timer timer = new Timer();


  /** Creates a new IntakeWithTime. */
  public IntakeWithTime() {
    // Use addRequirements() here to declare subsystem dependencies.

    intakeSubsystem = IntakeSubsystem.getInstance();
    addRequirements(intakeSubsystem);
        
    // Initialize the condition
    condition = intakeSubsystem.getCondition("autoActivated");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // When the command starts, tell the intake it can go
    condition.setState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When the command stops, tell the intake to not go
    AgitatorSubsystem.INSTANCE.stopAgitator();
    condition.setState(false);
  }
}
