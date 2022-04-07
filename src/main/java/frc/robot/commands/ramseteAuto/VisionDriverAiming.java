// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
    private PIDController m_pidController;
    private final double allowableVisionErrorMedian = 6.0;
    private final double allowableVisionErrorPredicted = 2.0;
    private final double clampPIDOutput = 0.5;

    /** Create all the fields */
    private final Supplier<Double> m_forwardVal;
    private final Supplier<Double> m_visionYaw;
    private final boolean m_runInAuto;
    private double m_targetYaw; 
    
    private CircularBuffer m_headingsBuffer = new CircularBuffer(4);
    private MedianFilter m_filterVisionYaw = new MedianFilter(8);
    private double m_prevVisionYaw = 0;
    private double m_prevHeading = 0;

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
    public VisionDriverAiming (Joystick driveJoystick, Supplier<Double> visionYawSupplier, boolean runInAuto) { 

        m_forwardVal = () -> sign(Config.removeJoystickDeadband(driveJoystick.getRawAxis(Config.LEFT_CONTROL_STICK_Y)), Config.INVERT_FIRST_AXIS);

        m_visionYaw = visionYawSupplier;
        m_runInAuto = runInAuto;
        
        m_pidController = new PIDController(0.014, 0.00001, 0.004);
        m_pidController.setIntegratorRange(0.2, 0.2);

        addRequirements(DriveBaseHolder.getInstance());
    }

    @Override
    public void initialize() {
        DriveBaseHolder.getInstance().setDriveMode(DriveBase.DriveMode.OpenLoopVoltage);
        DriveBaseHolder.getInstance().setNeutralMode(NeutralMode.Brake);
        m_prevHeading = -999;
        m_filterVisionYaw.reset();
        m_pidController.reset();
    }

    @Override
    public void execute() {
        double heading = DriveBaseHolder.getInstance().getOdometryHeading().getDegrees();
        double visionYaw = m_visionYaw.get();

        // Update the setpoint if vision check goes well
        if (visionYaw != -99) {
            double medianYaw = m_filterVisionYaw.calculate(visionYaw);

            boolean predictedYawCheck = true;
            if (m_prevHeading != -999) {
                predictedYawCheck = Math.abs(heading - m_prevHeading + m_prevVisionYaw - visionYaw) < allowableVisionErrorPredicted;
            }
            if (Math.abs(medianYaw-visionYaw) < allowableVisionErrorMedian && predictedYawCheck) {
                // Update the setpoint/target
                m_targetYaw = visionYaw + heading;

                m_prevHeading = heading;
                m_prevVisionYaw = visionYaw;
            }
            
        }

        double rotateValPid = m_pidController.calculate(heading, m_targetYaw);
        rotateValPid = MathUtil.clamp(rotateValPid, -clampPIDOutput, clampPIDOutput);

        DriveBaseHolder.getInstance().arcadeDrive(m_forwardVal.get(), rotateValPid, false);

        m_headingsBuffer.addFirst(heading);
    }

    @Override
    public void end(boolean interrupted) {
        DriveBaseHolder.getInstance().setNeutralMode(NeutralMode.Coast);
    }

    /**
     * When run in auto both conditions must be true to end command:
     *  - Yaw from vision is less than allowableYawError
     *  - The heading has changed less than allowableYawChangeToEnd in 0.08 seconds.
     * 
     * When run in telelop. Use the whileHeld to cancel this command when button is released.
     */
    @Override
    public boolean isFinished() {
        if (m_runInAuto) {
            double changeInHeading = m_headingsBuffer.get(0) - m_headingsBuffer.get(3);

            double visionYaw = m_visionYaw.get();
            boolean visionAligned = false;
            if (visionYaw != -99) 
                visionAligned = Math.abs(visionYaw) < allowableYawError;

            if (visionAligned && Math.abs(changeInHeading) < allowableYawChangeToEnd) {
                return true;
            }
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
