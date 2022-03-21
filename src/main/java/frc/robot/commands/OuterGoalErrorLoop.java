// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBaseHolder;

public class OuterGoalErrorLoop extends CommandBase {
    // Declare PD variables
    private Supplier<Double> pGain = Config.DRIVETRAIN_P;
    private Supplier<Double> dGain = Config.DRIVETRAIN_D; 

    // Weather or not too invert the direction turned (true if aiming the back of the robot)
    private boolean invert;

    // The maximum amount of time the command is allowed to run
    private Double maxTime;

    // The acceptable error for the target angle
    private Double acceptableError;
    private double timePointedAtTarget;
    private boolean isDone;

    private PigeonIMU pigeonIMU;
    
    private Timer timer = new Timer(); 

    public OuterGoalErrorLoop(boolean invert, Double acceptableError) {
        this.invert = invert;
        // this.maxTime = maxTime;
        this.acceptableError = acceptableError;

        pigeonIMU = DriveBaseHolder.getInstance().getPigeon();

        addRequirements(DriveBaseHolder.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Ensure the vision is running in tape mode
        VisionCtrlNetTable.setTapeMode();

        timer.reset();
        timer.start();

        isDone = false;
        timePointedAtTarget = 0;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the target angle from NetworkTables
        double currentAngle = VisionCtrlNetTable.yawToHub.get();

        // Check for no data code
        if (currentAngle == -99) {
            return;
        }
        // Filter out outlying values
        if(currentAngle <= -30 && currentAngle >= 30) {
            // Check if the yaw should be inverted (Shooter is on the back so we need to invert)
            return;
        }

        if (invert) {
            currentAngle *= -1;
        }

        // Get current angular rate
        double[] xyz_dps = new double[3];
        pigeonIMU.getRawGyro(xyz_dps);
        //Get z axis angular rate
        double currentAngularRate = xyz_dps[2];

        if (Math.abs(currentAngle) < acceptableError) {
            if (timePointedAtTarget == 0) {
                timePointedAtTarget = timer.get();
            } else if (timer.get() - timePointedAtTarget > 1.5) {
                isDone = true;
            }
        } else {
            timePointedAtTarget = 0;
        }

        //Do PD
        double turnThrottle = (currentAngle) * pGain.get() - (currentAngularRate) * dGain.get(); // Changed from + to -

        double forwardSpeed = 0;

        //Run motors according to the output of PD
        DriveBaseHolder.getInstance().tankDrive(-turnThrottle + forwardSpeed, turnThrottle + forwardSpeed, false);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
