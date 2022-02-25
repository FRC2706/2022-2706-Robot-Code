// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.config.Config;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.FluidConstant;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;


public class IndexerSubsystem extends SubsystemBase {

  private CANSparkMax m_indexer;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;

  // public static FluidConstant<Double> INDEXER_RPM_FOR_INDEXER = new FluidConstant<>
  //           ("IndexerRPM_forShooter", 400).registerToTable(Config.constantsTable);

  private static final IndexerSubsystem INSTANCE_INDEXER = new IndexerSubsystem();

  double kMaxOutput = 1;
  double kMinOutput = -1;

  double targetPosition = 2;
  double targetRPM = 500;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
        // Initialize the subsystem if the indexer exists
        if (Config.INDEXER_MOTOR != -1) {
          initializeSubsystem();
      }
      else
      {
          m_indexer = null;
      }

  }

    /**
     * Initialization process for the indexer to be run on robots with this
     * mechanism.
     */
    private void initializeSubsystem() {
      m_indexer = new CANSparkMax(Config.INDEXER_MOTOR, MotorType.kBrushless);

      // Factory Default to prevent unexpected behaviour
      m_indexer.restoreFactoryDefaults();

      // PID controller for the indexer
      m_pidController = m_indexer.getPIDController();
      m_encoder = m_indexer.getEncoder();

      m_indexer.setInverted(true);

      //slot 0: configuration for intake
      //        position closed loop
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);
      m_pidController.setFF(0.000156);
      m_pidController.setP(0);
      m_pidController.setI(0);
      m_pidController.setD(0);


      //slot 1: configuration for shooter.
      //        velocity closed loop
      m_pidController.setOutputRange(kMinOutput, kMaxOutput, 1);
      m_pidController.setFF(0.000156, 1);
      m_pidController.setP(0, 1);
      m_pidController.setI(0, 1);
      m_pidController.setD(0, 1);


      m_indexer.setSmartCurrentLimit(60);

  }

  public boolean isActive() {
    return m_indexer != null;
    }

    /**
     * Returns the singleton instance for the indexerSubsystem
     */
    public static IndexerSubsystem getInstance() {
      if (INSTANCE_INDEXER.isActive())
          return INSTANCE_INDEXER;
      else
          return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunForIntake()
  {
    //get the current position
    double currPosition = m_encoder.getPosition();
    m_pidController.setReference(currPosition + targetPosition, ControlType.kPosition);

  }

  public void RunForShooter()
  {
    m_pidController.setReference(targetRPM, ControlType.kVelocity, 1);
  }

  //@todo: avoid running for intake and for shooter at the same time.
  //       the simplest way is just not pressing two buttons at the same time.

}
