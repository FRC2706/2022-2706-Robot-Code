// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PollLimitSwitch extends CommandBase {
	Command m_commandToSchedule;
	SubsystemBase m_subsystem;
	Supplier<Boolean> m_limitSwitchActive;

	/**
	 * This command will poll the limit switch at the input of the feeder and
	 * shedule a command to do an action.
	 * 
	 * This command is meant to be the default command of a subsystem.
	 * 
	 * This is useful because instead of putting it in the periodic of a subsystem,
	 * this command will get unsheduduled if the subsytem is needed for a different
	 * action.
	 */
	public PollLimitSwitch(Command commandToSchedule, SubsystemBase subsystem, Supplier<Boolean> limitSwitchActive) {
		this.m_commandToSchedule = commandToSchedule;
		this.m_subsystem = subsystem;
		this.m_limitSwitchActive = limitSwitchActive;

		addRequirements(subsystem);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_limitSwitchActive.get()) {
			this.cancel();
			m_commandToSchedule.schedule();
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
