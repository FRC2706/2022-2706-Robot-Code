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

  //@todo: move to Config
  //Hub center coordinates in meters
  double m_hubX = 8.23;
  double m_hubY = 4.115;

  double m_theta;
  double m_deltaTheta;
  double m_distance;

  // Get the drivebase and pigeon
  private final DriveBase m_drivebase;
  
  //A timer to ensure the command doesn't get stuck and the robot cannot drive
  private Timer m_timer;
  private double m_timeout;

  private boolean bDone;
  private boolean bTimeouted;
  private final double m_errMeters = 0.01;//in meter, 1cm

  // USB Logger
  private boolean bUsbLogger = false;
  private SimpleCsvLogger usbLogger;
  private String loggingDataIdentifier = "DrivetrainAlignment";

  public DrivetrainAlignment() {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivebase = DriveBaseHolder.getInstance();
    addRequirements(m_drivebase);

    m_timer = new Timer();
    m_timeout = 1.5;  //seconds //@todo: from config
    
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
    //Calculate the degree that the robot needs to turn
    calcDeltaDegree();
    //Map the degree to distance in meters
    covertDegreeToPositionMeter();

    m_targetLeftPositionMeter  = m_drivebase.getLeftPosition() - m_targetDeltaPositionMeter;
    m_targetRightPositionMeter = m_drivebase.getRightPosition() + m_targetDeltaPositionMeter;

    m_initDegree = m_drivebase.getOdometryHeading().getDegrees();
    m_targetDegree = m_initDegree + m_deltaDegree;

    //setup PID slot of two master talons
    m_drivebase.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_ALIGNMENT);
    m_drivebase.setCoastMode();

    m_timer.start();
    m_timer.reset();

    bDone = false;
    bTimeouted = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Since targe position is fixed, we can set it multiple times
    m_drivebase.tankDrivePosition( m_targetLeftPositionMeter, m_targetRightPositionMeter);

    //get the current encoder positions
    m_currLeftPosMeter =  m_drivebase.getLeftPosition();
    m_currRightPosMeter = m_drivebase.getRightPosition();

    //@todo: this may not be accurate. The reason is after the command finishes, the robot still may move.
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
    double finalDegree =  m_drivebase.getPose().getRotation().getDegrees();
    double turnedDegree = finalDegree - m_initDegree;
    double errDegree = m_targetDegree - finalDegree;
    // System.out.println("current odometry angle (degrees): "+ finalDegree);
    // System.out.println("target odometry angle (degrees): " + m_targetDegree);
    System.out.println("turned angle (degrees): "+ turnedDegree);
    System.out.println("init: " + m_initDegree+" final: "+ finalDegree );
    double errLeftPos = -m_targetLeftPositionMeter + m_currLeftPosMeter;
    double errRightPos = m_targetRightPositionMeter - m_currRightPosMeter;
    //note: error > 0 means underrun
    System.out.println("pos errs: " + errLeftPos + " " + errRightPos);
    System.out.println("time: " + m_timer.get());
    System.out.println("desired theta: "+ m_theta);
    System.out.println("m_DeltaTheta: "+m_deltaTheta + " distance: "+ m_distance);
    System.out.println(" ");
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
    //convert m_deltaDegree to m_targetPositionMeter based on radius.
    
    //for test only, m_distance(m)
    //m_distance = 0.10;

    //use the arc formula to calcuate
    //note: m_distance could be negative.
    // m_distance = m_deltaTheta*3.14*Config.drivetrainRotateDiameter/360.0;
   
    //use the trend line: deltaTheta --> distance
    //Beetle in basement
    //m_distance = Math.signum(m_deltaTheta)*(0.00205*Math.abs(m_deltaTheta)+0.0601);

    //use the trend polynomial (Beetle in basement)
    m_distance = Math.signum(m_deltaTheta)*(0.0592 + 0.00209*Math.abs(m_deltaTheta) - 0.000000171*m_deltaTheta*m_deltaTheta);

    //Beetle on competition carpet
    //m_distance =  Math.signum(m_deltaTheta)*(0.00309*Math.abs(m_deltaTheta)+0.0068);

    m_targetDeltaPositionMeter = m_distance;
  }

  /*
   * This method maps deltaTheta to distance
   */
  public void calcDeltaDegree()
  {
    //get current coordinate
    Pose2d currPose = m_drivebase.getPose();
    double x = currPose.getX();
    double y = currPose.getY();
       
    if ( x >= m_hubX )
    {
      if ( y >= m_hubY )
      {
        //quadrant III
        double thetaPrime = Math.atan((x-m_hubX)/(y-m_hubY));
        m_theta = -(180/3.14)*thetaPrime - 90;
      }
      else
      {
        //quadrant II
        double thetaPrime = Math.atan((x-m_hubX)/(m_hubY-y));
        m_theta = 90 + (180/3.14)*thetaPrime;
      }

    }
    else //x <= m_hubX
    {
      if ( y >= m_hubY )
      {
        //quadrant IV
        double thetaPrime = Math.atan((m_hubX-x)/(y-m_hubY));
        m_theta = (180/3.14)*thetaPrime - 90;
      }
      else
      {
        //quadrant I
        double thetaPrime = Math.atan((m_hubX-x)/(m_hubY-y));
        m_theta = 90 - (180/3.14)*thetaPrime;
      }

    }
    
    //note:: m_deltaTheta > 0 --> counter clockwise rotation
    //       m_deltaTheta < 0 --> clockwise rotation
    m_deltaTheta = m_theta - currPose.getRotation().getDegrees();
    
    //make sure m_deltaTheta is [-180, +180]
    if ( m_deltaTheta > 180 )
    {
      m_deltaTheta -= 360;
    }
    else if ( m_deltaTheta < -180 )
    {
      m_deltaTheta += 360;
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
                             "bDone",
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
