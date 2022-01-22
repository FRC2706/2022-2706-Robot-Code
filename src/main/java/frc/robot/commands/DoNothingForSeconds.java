// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoNothingForSeconds extends CommandBase {
    private final Timer m_timer = new Timer();
    double m_timeSeconds;

    /**
     * Creates a new DoNothingForSeconds.
     * 
     * This command will make a subsystem wait for a time given.
     */
    public DoNothingForSeconds(double timeSeconds, SubsystemBase... subsystemRequirements) {
        m_timeSeconds = timeSeconds;
        addRequirements(subsystemRequirements);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_timeSeconds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
