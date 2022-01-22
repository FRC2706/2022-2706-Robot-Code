// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConditionalSubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private ConditionalSubsystemBase.SubsystemCondition condition;

    public AutoIntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(intakeSubsystem);

        // Initialize the condition
        condition = intakeSubsystem.getCondition("autoActivated"); 
    }

    @Override
    public void initialize() {
        // When the command starts, tell the intake it can go
        condition.setState(true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        condition.setState(false);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
