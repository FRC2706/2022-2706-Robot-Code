// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.config.Config;
import frc.robot.commands.ramseteAuto.SimpleCsvLogger;

import edu.wpi.first.math.geometry.Pose2d;


public class DrivetrainAlignment extends CommandBase {
  /** Creates a new DrivetrainAlignment. */
  double m_deltaDegree;
  double m_initDegree;
  double m_targetDegree;

  double m_targetDeltaPositionMeter;
  double m_targetLeftPositionMeter;
  double m_targetRightPositionMeter;
  double m_currLeftPosMeter;
  double m_currRightPosMeter;

  double m_hubX = 1.8;
  double m_hubY = 2.5;
  double m_theta;
  double m_distance;
  double m_deltaTheta;

  // Get the drivebase and pigeon
  private final DriveBase m_drivebase;
  
  //A timer to ensure the command doesn't get stuck and the robot cannot drive
  private Timer m_timer;
  private double m_timeout;

  private boolean bDone;
  private boolean bTimeouted;
  private final double m_errMeters = 0.01;// 1cm

  // USB Logger
  private boolean bUsbLogger = false;
  private SimpleCsvLogger usbLogger;
  private String loggingDataIdentifier = "DrivetrainAlignment";

  public DrivetrainAlignment(double deltaDegree) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_deltaDegree = deltaDegree;

    m_drivebase = DriveBaseHolder.getInstance();
    addRequirements(m_drivebase);

    m_timer = new Timer();
    m_timeout = 2.0;//1.0;//0.5;//0.25;        //seconds //@todo: from config
    
    System.out.println("DrivertrainAlignemnt construct");

    if ( bUsbLogger == true )
    {
      usbLogger = new SimpleCsvLogger();
    }
    else
    {
      usbLogger = null;
    }

    if ( bUsbLogger == true )
    {
      startLogging();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calcDeltaDegree();
    covertDegreeToPositionMeter();

    //Note: always counter-clockwise rotation. Keep this in mind when calcualte target angle
    m_targetLeftPositionMeter  = m_drivebase.getLeftPosition() - m_targetDeltaPositionMeter;
    m_targetRightPositionMeter = m_drivebase.getRightPosition() + m_targetDeltaPositionMeter;

    m_initDegree = m_drivebase.getOdometryHeading().getDegrees();
    m_targetDegree = m_initDegree + m_deltaDegree;

    //setup PID slot of two master talons
    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_ALIGNMENT);
    m_drivebase.setCoastMode();

    m_timer.start();
    //todo: this reset has to be added.
    m_timer.reset();
    bDone = false;
    bTimeouted = false;

//    System.out.println("DrivertrainAlignemnt initialize " + m_targetLeftPositionMeter  +" "+ m_targetRightPositionMeter );
 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Since targe position is fixed, we can set it multiple times
    m_drivebase.tankDrivePosition( m_targetLeftPositionMeter, m_targetRightPositionMeter);

    //get the current encoder positions
    m_currLeftPosMeter =  m_drivebase.getLeftPosition();
    m_currRightPosMeter = m_drivebase.getRightPosition();

    if ( (Math.abs(m_currLeftPosMeter - m_targetLeftPositionMeter) < m_errMeters )
         && ( Math.abs(m_currRightPosMeter - m_targetRightPositionMeter) < m_errMeters ))
    {
      bDone = true;
      // System.out.println("left error abs: " + Math.abs(m_currLeftPosMeter - m_targetLeftPositionMeter));
      // System.out.println("right error abs: "+ Math.abs(m_currRightPosMeter - m_targetRightPositionMeter));
    }

  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_DRIVER);

    //logging the final current heading and error angle
    double finalDegree = m_drivebase.getOdometryHeading().getDegrees();
    double errDegree = finalDegree - m_targetDegree;
    // System.out.println("current odometry angle (degrees): "+ finalDegree);
    // System.out.println("target odometry angle (degrees): " + m_targetDegree);
    System.out.println("error angle (degrees): "+ errDegree);
    double errLeftPos = -m_targetLeftPositionMeter + m_currLeftPosMeter;
    double errRightPos = m_targetRightPositionMeter - m_currRightPosMeter;
    //note: error > 0 means underrun
    System.out.println("pos errs: " + errLeftPos + " " + errRightPos);
    System.out.println("time: " + m_timer.get());
    System.out.println("theta: "+m_theta+" distance: "+m_distance);
    System.out.println("m_DeltaTheta: "+m_deltaTheta);
    //@todo: after target position is reached, stop the cmd.
    //@max velocity, trapezoid control
    //System.out.println("curr left pos: "+ m_currLeftPosMeter+" target pos: "+ m_targetLeftPositionMeter);
    // System.out.println("curr right pos: "+ m_currRightPosMeter+" target pos: "+ m_targetRightPositionMeter);
    // System.out.println("timeout: " + bTimeouted + " current time " + m_timer.get());
    // System.out.println("bDone: " + bDone);

    
    double timeouted = 0.;
    double done = 0.;
    if( bTimeouted == true)
      timeouted = 1.0;

    if ( bDone == true )
       done = 1.0;


    if( bUsbLogger == true)
    {
      logData(finalDegree,
              m_targetDegree,
              errDegree,
              m_currLeftPosMeter,
              m_targetLeftPositionMeter,
              m_currRightPosMeter,
              m_targetRightPositionMeter,
              m_timer.get(),
              timeouted,
              done);
    }

    // if( bUsbLogger == true)
    // {
    //   stopLogging();
    // }

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ( m_timer.get() > m_timeout )
    {
      bTimeouted = true;
    }

    if( bTimeouted == true || bDone == true)
      return true;
    else
      return false;
  }

  public void covertDegreeToPositionMeter()
  {
    //@todo: convert m_deltaDegree to m_targetPositionMeter based on radius.
    m_targetDeltaPositionMeter = m_distance;//0.246;//0.45;//0.246;//0.246;  //mapped to 90 degrees
  }

  public void calcDeltaDegree()
  {
    //get current coordinate
    Pose2d currPose = m_drivebase.getPose();
    double x = currPose.getX();// m_drivebase.getLeftPosition();
    double y = currPose.getY();

    double thetaPrime = Math.atan((x-m_hubX)/(m_hubY-y));

    m_theta = (180/3.14)*thetaPrime+90;

    m_deltaTheta = m_theta-currPose.getRotation().getDegrees();

    if(Math.abs(m_deltaTheta) < 90){
      m_distance = sign(m_deltaTheta) * (0.00246*Math.abs(m_deltaTheta)+0.0526);
    }
    else{
      m_distance = sign(m_deltaTheta)*(-0.161 + 0.00624*Math.abs(m_deltaTheta) -0.0000147*m_deltaTheta*m_deltaTheta);
    }
  }

  public double sign(double input)
  {
    if(input > 0)
    {
      return 1.00;
    }
    else{
      return -1.00;
    }
  }

    /**
     * All Methods used for USB logging startLogging() logData(data) stopLogging()
     */
  public void startLogging() {
      // See Spreadsheet link at top
      usbLogger.init(loggingDataIdentifier, 
              new String[] { "currAngle",
                             "targetAngle",
                             "errAngle",
                             "currLeftPos",
                             "targetLeftPos",
                             "currRightPos",
                             "targetRightPos",
                             "time",
                             "bDOne",
                             "bTimeouted"},
              new String[]{"deg",
                           "deg",
                           "deg",
                           "m",
                           "m",
                           "m",
                           "m",
                           "sec",
                           "bool",
                           "bool"});
  }

  public void logData(double... data) {
        usbLogger.writeData(data);
  }

  public void stopLogging() {
        usbLogger.close();
  }
}
