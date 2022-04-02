/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.config.Config;
import frc.robot.subsystems.*;

/**
 * Triggers a modifier which limits the speed the robot can turn and only allows it to turn in place
 */
public class SensitiveDriverControl extends CommandBase {
    
    //get the drivebase
    private DriveBase driveBase;
    private Joystick joystick;

    private final Supplier<Double> m_forwardVal;
    private final Supplier<Double> m_rotateVal;

    public SensitiveDriverControl(Joystick joystick) {
        //set the drivebase
        this.driveBase = DriveBaseHolder.getInstance();
        addRequirements(this.driveBase);
        this.joystick = joystick;

        //Separation of concern- no other class uses this code
        new JoystickButton(joystick, XboxController.Button.kLeftBumper.value).whenHeld(this);        

        m_forwardVal = () -> sign(Config.removeJoystickDeadband(joystick.getRawAxis(Config.LEFT_CONTROL_STICK_Y)), Config.INVERT_FIRST_AXIS);
        m_rotateVal = () -> sign(Config.removeJoystickDeadband(joystick.getRawAxis(Config.RIGHT_CONTROL_STICK_X)), Config.INVERT_SECOND_AXIS);
    }

    /**
     * Drives with sensitive control
     */
    @Override
    public void execute() {
        this.driveBase.setSensitiveSteering(true);
        // this.driveBase.tankDrive(this.joystick.getRawAxis(Config.RIGHT_CONTROL_STICK_X)*Config.DRIVETRAIN_SENSITIVE_MAX_SPEED.get(), -this.joystick.getRawAxis(Config.RIGHT_CONTROL_STICK_X)*Config.DRIVETRAIN_SENSITIVE_MAX_SPEED.get(), false);
        driveBase.arcadeDrive(m_forwardVal.get()*Config.DRIVETRAIN_SENSITIVE_FORWARD_SPEED, m_rotateVal.get()*Config.DRIVETRAIN_SENSITIVE_ROTATE_SPEED, true);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    /**
     * Stops the drivebase
     */
    @Override
    public void end(boolean interrupted) {
        this.driveBase.setSensitiveSteering(false);
        this.driveBase.stopMotors();
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