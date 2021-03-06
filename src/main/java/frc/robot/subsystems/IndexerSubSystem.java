// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.AutomaticShooter;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubSystem extends SubsystemBase {
  
  public static FluidConstant<Double> INDEXER_RPM = new FluidConstant<>
  ("Indexer_PRM",2000.).registerToTable(Config.constantsTable);

  //Instance Variables
  private CANSparkMax m_indexer;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double targetRPM = INDEXER_RPM.getValue();//700;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double currentPosition = 0;
  public boolean m_bForShooterGoodSensors= false;
  public boolean m_bForIntakeGoodSensors = false;
  private static final IndexerSubSystem INSTANCE_INDEXER = new IndexerSubSystem();


  //@todo: put these values as constant configs for competition
    //       or make them configurable from the network table (network listener for debug)
    // PID values (currently set for protobot's shooter)
    public static FluidConstant<Double> P_INDEXERSUBSYSTEM = new FluidConstant<>
            ("P_IndexerSubsystem", 3.5e-4).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> I_INDEXSUBSYSTEM = new FluidConstant<>
            ("I_IndexerSubsystem", 3.7e-7).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> D_INDEXERSUBSYSTEM = new FluidConstant<>
            ("D_IndexerSubsystem", 0.013).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> F_INDEXERSUBSYSTEM = new FluidConstant<>
            ("F_IndexerSubsystem", 0.000175).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> IZONE_INDEXERSUBSYSTEM = new FluidConstant<>
            ("IZONE_IndexerSubsystem", 150.0).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> TARGET_RPM = new FluidConstant<>
            ("TARGET_RPM_IndexerSubsystem", 2000.0).registerToTable(Config.constantsTable);
    //note: during the test to find a good RPM, use TARGET_RPM in setTargetRPM(int inputRPM).

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

  private void initializeSubsystem() 
  {
    m_indexer = new CANSparkMax(Config.INDEXER_MOTOR, MotorType.kBrushless);

    if ( m_indexer != null )
    {      
      m_bForIntakeGoodSensors = true;
      m_bForShooterGoodSensors = true;
  
      // Factory Default to prevent unexpected behaviour
      m_indexer.restoreFactoryDefaults();
      m_indexer.setInverted(false);

      // PID controller for the shooter
      m_pidController = m_indexer.getPIDController();
      m_encoder = m_indexer.getEncoder();

      if ( m_pidController == null || m_encoder == null )
      {
        m_bForShooterGoodSensors = false;
        m_bForIntakeGoodSensors = false;
        return;
      }

      REVLibError errorCode;
         
      //Set for intake slot 0
      //Use smart position closed loop controller
      errorCode = m_pidController.setOutputRange(kMinOutput, kMaxOutput);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setFF(0.0005);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setP(5e-5);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setI(1e-6);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setD(1e-6);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setIZone(100);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
        
      //Set max acceleration, velocity, and minimum velocity
      errorCode = m_pidController.setSmartMotionMaxAccel(1000, 0);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setSmartMotionMaxVelocity(1200, 0);
      m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);
      
      // the following line cause the trouble: make m_bForIntakeGoodSensors to false
      //  errorCode = m_pidController.setSmartMotionMinOutputVelocity(-1, 0);
      // m_bForIntakeGoodSensors = m_bForIntakeGoodSensors && (errorCode == REVLibError.kOk);

      //Set for shooter slot 1
      //Use velocity closed loop controller
      errorCode = m_pidController.setOutputRange(kMinOutput, kMaxOutput, 1);
      m_bForShooterGoodSensors = m_bForShooterGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setFF(0.000156,1);
      m_bForShooterGoodSensors = m_bForShooterGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setP(5e-5,1);
      m_bForShooterGoodSensors = m_bForShooterGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setI(1e-6,1);
      m_bForShooterGoodSensors = m_bForShooterGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setD(1e-6,1);
      m_bForShooterGoodSensors = m_bForShooterGoodSensors && (errorCode == REVLibError.kOk);
      errorCode = m_pidController.setIZone(100,1);
      m_bForShooterGoodSensors = m_bForShooterGoodSensors && (errorCode == REVLibError.kOk);

      //Set maximum current
      m_indexer.setSmartCurrentLimit(60);
    }
 }

    public boolean isActive() 
    {
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

    public void setIndexerPosition()
    {
      if( m_bForIntakeGoodSensors == true )
      { 
        //Get the current motor position from encoder
        currentPosition = m_encoder.getPosition();
      }
    }

    //Run the intake
    public void runForIntake( int increPosition) {

      if( m_bForIntakeGoodSensors == true )
      {
        //Use smartmotion to go from current position to new position
        m_pidController.setReference(currentPosition + increPosition, ControlType.kSmartMotion, 0);

        //use the finxed incrementalPosition
        //m_pidController.setReference(currentPosition + incrementalPosition, ControlType.kSmartMotion, 0);

        //Use arbitrary feed forward: 0.5 voltage
        //note: arbitraryFF could be dependent on if there is already one cargo in the indexer
        //m_pidController.setReference(currentPosition + incrementalPosition, ControlType.kSmartMotion, 0, 0.5, com.revrobotics.SparkMaxPIDController.ArbFFUnits.kVoltage);
      }
      else
      {
        //a constant speed, then only for one cargo
        //@todo: test this default speed
        m_indexer.set(0.1);
      }

    }

    //Run the shooter
    public void runForShooter() {
      double rpm = ShooterSubsystem.TARGET_RPM.getValue();
      if(rpm > 0)
      {
        //to run with the calculated target RPM
        if ( m_bForShooterGoodSensors == true )
        {
          m_pidController.setReference(targetRPM, ControlType.kVelocity, 1);
        }
        else
        {
          //@todo: don't use the PIDF. Need to tune for a good percentage for the shooter.
          //700/4000.0
          m_indexer.set(0.2);
        }
      }
      else
      {
        stop();
      } 
    }
    
    //To run with the input RPM, for taking only one cargo in
    public void setTargetRPM(double inputRPM)
    {
      if( m_bForShooterGoodSensors == true )
        m_pidController.setReference(inputRPM, ControlType.kVelocity, 1);
      else
      {
        //@todo: need to calibrate the max RPM with 100 percentage.
        // for now, assume it is 4000.0
        m_indexer.set(inputRPM/4000.0);
        
      }
    }

    public void stop() 
    {
      m_indexer.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
