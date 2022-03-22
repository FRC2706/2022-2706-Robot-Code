// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class IntakeCargoSubsystem extends SubsystemBase {

  private TalonSRX m_intake;
  ErrorCode errorCode;

  private final static IntakeCargoSubsystem INSTANCE_INTAKE = new IntakeCargoSubsystem(); 

  /** Creates a new IntakeCargoSubsystem. */
  public IntakeCargoSubsystem() 
  {
    // Initialize the subsystem if the intake exists
    if (Config.INTAKE_MOTOR != -1) {
      initializeSubsystem();
    }
    else
    {
      m_intake = null;
    }
  }

  public void initializeSubsystem()
  {
    // Init all the talon values
    m_intake = new WPI_TalonSRX(Config.INTAKE_MOTOR);

    if ( m_intake != null )
    { 
      // Config factory default to clear out any lingering values
      m_intake.configFactoryDefault();

      // Setup the talon, recording the error code
      // errorCode = m_intake.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
      //         0, Config.CAN_TIMEOUT_SHORT);

      //@todo: need testing
      m_intake.setInverted(Config.INVERT_ARM_TALON);
    }

    //m_intake.configAllowableClosedloopError(0, Config.ARM_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS, Config.CAN_TIMEOUT_SHORT);

    /*
    //  Config the PID Values based on constants
    m_intake.config_kP(0, Config.ARM_PID_P, Config.CAN_TIMEOUT_LONG);
    m_intake.config_kI(0, Config.ARM_PID_I, Config.CAN_TIMEOUT_LONG);
    m_intake.config_kD(0, Config.ARM_PID_D, Config.CAN_TIMEOUT_LONG);
    m_intake.config_kF(0, Config.ARM_PID_F, Config.CAN_TIMEOUT_LONG);
    m_intake.config_IntegralZone(0, Config.ARM_PID_IZONE, Config.CAN_TIMEOUT_LONG);
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static IntakeCargoSubsystem getInstance() {
    if (INSTANCE_INTAKE.isActive())
        return INSTANCE_INTAKE;
    else
        return null;
  }

  public boolean isActive() {
    return m_intake != null;
  }

  public void start()
  {
    //@todo: need to adjust 0.2
    //@todo: 
    m_intake.set(ControlMode.PercentOutput, 0.8);
  }

  public void stop()
  {
    m_intake.set(ControlMode.PercentOutput, 0);
  }

}
