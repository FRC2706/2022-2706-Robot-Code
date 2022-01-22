// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.FeederSubsystem;

public class IndexBall extends CommandBase {
	
	FeederSubsystem feeder;
	int targetPositon;
	int incrementTicks;
	boolean reversing;
	boolean isDone;

	boolean maxBalls;
	boolean outputSwitchPressed;

	private final boolean setTalonPosistionEveryCycle = true;
	

	public IndexBall() {
		
		feeder = FeederSubsystem.getInstance();

		addRequirements(feeder);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		incrementTicks = -1 * Config.FEEDERSUBSYSTEM_INCREMENT_TICKS.get().intValue();

		int currentPosition = (int) feeder.getCurrentPosition();
		targetPositon = currentPosition + incrementTicks;

		reversing = false;
		isDone = false;

		if (feeder.getBallsAroundFeeder() >= Config.FEEDER_MAX_BALLS) {
			maxBalls = true;
			this.cancel();
			return;
		} else if (FeederSubsystem.isBallAtOutput()) {
			outputSwitchPressed = true;
			this.cancel();
			return;
		}

		maxBalls = false;
		outputSwitchPressed = false;
		feeder.setFeederPosistion(targetPositon);
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (maxBalls == true) {
			isDone = true;
			return;
		} else if (FeederSubsystem.isBallAtOutput()) {
			outputSwitchPressed = true;
			isDone = true;
			return;
		}

		int currentPosition = (int) feeder.getCurrentPosition();
		boolean atPosistion = feeder.isFeederAtPosistion(Config.FEEDERSUBSYSTEM_INDEX_ALLOWABLE_ERROR);
		int ticksChanged = currentPosition - (targetPositon - incrementTicks);
		
		if (currentPosition >= targetPositon || atPosistion) {
			isDone = true;
		} else if (reversing) {
			if (atPosistion) {
				isDone = true;
			} else if(setTalonPosistionEveryCycle) {
				feeder.setFeederPosistion(targetPositon - incrementTicks);
			}
		// } else if (feeder.isBallAtInput() == false && ticksChanged < Config.FEEDERSUBSYSTEM_POS_PAST_SWITCH) {
			// Limit unpressed & in range that it should be pressed then reverse, likely ball bounced away
			// feeder.setFeederPosistion(targetPositon - incrementTicks);
			// reversing = true;
		} else if (setTalonPosistionEveryCycle) {
			feeder.setFeederPosistion(targetPositon);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (reversing == false && maxBalls == false && outputSwitchPressed == false) {
			feeder.setBallsAroundFeeder(feeder.getBallsAroundFeeder() + 1);
		}

		if (outputSwitchPressed || FeederSubsystem.isBallAtOutput()) {
			feeder.setFeederPosistion((int) feeder.getCurrentPosition());
			feeder.stopFeeder();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return isDone;
	}
}
