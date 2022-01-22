// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveBaseHolder;

public class PrintOdometry extends InstantCommand {
    public PrintOdometry() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d pose = DriveBaseHolder.getInstance().getPose();
        System.out.println(String.format("new PoseScaled(%.3f, %.3f, %.3f)", pose.getX(), pose.getY(),
                pose.getRotation().getDegrees()));
    }
}
