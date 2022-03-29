// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.CircularBuffer;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

public class VisionDriverAiming extends CommandBase {

    /**
     * CONSTANTS
     */
    // Angles in degrees
    private final double allowableYawError = 2.0;
    private final double allowableYawChangeToEnd = 2.0;
    private PIDController m_pidController = new PIDController(0.046, 0, 0.05);

    /** Create all the fields */
    private final Supplier<Double> m_forwardVal;
    private final Supplier<Double> m_visionYaw;
    private double m_targetYaw;
    private CircularBuffer m_headingsBuffer = new CircularBuffer(4);
    

    /**
     * Vision Aiming
     * Runs a PID loop with Vision data to control the rotation val of arcade drive.
     * Allows the forward val of arcade to still be driver controlled
     * 
     * Construct in the RobotContainer.java as shown below (for 2022):
     *  - Command cmdVisionAiming = new VisionAiming(driverStick, VisionCtrlNetTable.yawToHub);
     *  - Command cmdVisionAiming = new VisionAiming(driverStick, VisionCtrlNetTable.yawToCargo);
     *
     * @param driveJoystick Driver Joystick from RobotContainer
     * @param visionYawSupplier Supplier to get yaw from vision
     */
    public VisionDriverAiming (Joystick driveJoystick, Supplier<Double> visionYawSupplier) { 

        m_forwardVal = () -> sign(Config.removeJoystickDeadband(driveJoystick.getRawAxis(Config.LEFT_CONTROL_STICK_Y)), Config.INVERT_FIRST_AXIS);

        m_visionYaw = visionYawSupplier;

        addRequirements(DriveBaseHolder.getInstance());
    }

    @Override
    public void initialize() {
        DriveBaseHolder.getInstance().setDriveMode(DriveBase.DriveMode.OpenLoopVoltage);
        DriveBaseHolder.getInstance().setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void execute() {
        double heading = DriveBaseHolder.getInstance().getOdometryHeading().getDegrees();
        double visionYaw = m_visionYaw.get();

        if (visionYaw != -99) {
            m_targetYaw = visionYaw + heading;
        }

        double rotateValPid = m_pidController.calculate(heading, m_targetYaw);

        DriveBaseHolder.getInstance().arcadeDrive(m_forwardVal.get(), rotateValPid, false);

        m_headingsBuffer.addFirst(heading);
    }

    @Override
    public void end(boolean interrupted) {
        DriveBaseHolder.getInstance().setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Both condition must be true to end command:
     *  - Yaw from vision is less than allowableYawError
     *  - The heading has changed less than allowableYawChangeToEnd in 0.08 seconds.
     */
    @Override
    public boolean isFinished() {
        double changeInHeading = m_headingsBuffer.get(0) - m_headingsBuffer.get(3);

        if (Math.abs(m_visionYaw.get()) < allowableYawError && Math.abs(changeInHeading) < allowableYawChangeToEnd) {
            return true;
        }
        return false;
    }

   /**
   * Stolen From ArcadeDriveWithJoystick.java
   * Inverts a given double
   *
   * @param number The original number
   * @param sign Weather or not to invert it
   * @return The inverted value
   */
  private static double sign(double number, boolean sign){
    if(sign){
      return -number;
    }
    else{
      return number;
    }
  }
}
