// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.config.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubSystem extends SubsystemBase {
  
  //Instance Variables
  private CANSparkMax m_indexer;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double targetRPM = 200;
  private double incrementalPosition = 2;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  private static final IndexerSubSystem INSTANCE_INDEXER = new IndexerSubSystem();

  /** Creates a new IndexerSubSystem. */
  private IndexerSubSystem() {
    
    if (Config.INDEXER_MOTOR != -1) {
      initializeSubsystem();
    }
    else
    {
      m_indexer = null;
    }

  }

  private void initializeSubsystem() {
    
    m_indexer = new CANSparkMax(Config.INDEXER_MOTOR, MotorType.kBrushless);

        // Factory Default to prevent unexpected behaviour
        m_indexer.restoreFactoryDefaults();

        // PID controller for the shooter
        m_pidController = m_indexer.getPIDController();
        m_encoder = m_indexer.getEncoder();

        m_indexer.setInverted(true);

        
        //Set for intake slot 0
        //Use smart position closed loop controller
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setFF(0);
        m_pidController.setP(0);
        m_pidController.setI(0);
        m_pidController.setD(0);
        //Set max acceleration, velocity, and minimum velocity
        m_pidController.setSmartMotionMaxAccel(0, 0);
        m_pidController.setSmartMotionMaxVelocity(0, 0);
        m_pidController.setSmartMotionMinOutputVelocity(0, 0);

        //Set for shooter slot 1
        //Use velocity closed loop controller
        m_pidController.setOutputRange(kMinOutput, kMaxOutput, 1);
        m_pidController.setFF(0,1);
        m_pidController.setP(0,1);
        m_pidController.setI(0,1);
        m_pidController.setD(0,1);

        //Set maximum current
        m_indexer.setSmartCurrentLimit(60);
    }

    public boolean isActive() {
        return m_indexer != null;
    }

     /*
     * Returns the singleton instance for the Indexer Subsystem
     */
    public static IndexerSubSystem getInstance() {
      if (INSTANCE_INDEXER.isActive()) { 
          return INSTANCE_INDEXER;
      }
      else {
          return null;
      }
    }

    //Run the intake
    public void runForIntake() {
      
      //Get the current motor position from encoder
      double currentPosition = m_encoder.getPosition();

      //Use smartmotion to go from current position to new position
      m_pidController.setReference(currentPosition + incrementalPosition, ControlType.kSmartMotion, 0);

    }

    //Run the shooter
    public void runForShooter() {


      m_pidController.setReference(targetRPM, ControlType.kVelocity, 1);

    }
    public void stop() {
      m_indexer.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
