// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.shooterPneumaticSubsystem;

public class AutomaticShooter extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private shooterPneumaticSubsystem deflectorSubsystem;
  private NetworkTableEntry tableTargetRPM, tableMeasuredRPM, tableErrorRPM, tableTemperature;
  private NetworkTableEntry tableCurrent, tableIsAtTargetRPM, tableTargetV, tableTargetDistance;
  private NetworkTableEntry visionDistance;

  //calculated for the shooter
  double targetDistance = 0;
  int targetRPM = 0;

  private Timer timer;
  private int timeout = 5; //sec
  private boolean m_bUseTimer = false;

  //todo: can be configured in config file as well
  //todo: measure the radius for the shooting wheel

  //todo: another shooter angle: 70 with deflector on
  private final double SHOOTER_ANGLE_DEG = 60.0;
  private final double SHOOTER_ANGLE_DEG_DEFLECTOR = 70.0;
  //high goal heigth = 2.64m, low goal heigth = 1.04m
  private final double HIGH_GOAL_HEIGHT_IN_METERS = 2.64;
  private final double LOW_GOAL_HEIGHT_IN_METERS = 1.04;

  private final double SHOOTER_WHEEL_RADIUS_IN_CM = 7.62; //6 inch diameter
  private final double HALF_OF_GRAVITY = 4.91;
  private final double CONVERSION_NUMBER = 3000;
  
  private final double HUB_X = Config.HUB_X;
  private final double HUB_Y = Config.HUB_Y;

  private double targetHeight;
  private double shooterAngle;

  private boolean m_bHighGoal;
  private boolean m_bDeflector;
  private boolean m_bVision;

  private NetworkTableEntry currentX, currentY;

  /** Creates a new AutomaticShooter. */
  public AutomaticShooter(boolean bVision) {
    m_bVision = bVision;
    
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = ShooterSubsystem.getInstance();

    //@todo: add deflector subsystem later
    //deflectorSubsystem = shooterPneumaticSubsystem.getInstance();
    deflectorSubsystem = null;

    if (shooterSubsystem != null) 
    {
      addRequirements(shooterSubsystem);
    }
    if (deflectorSubsystem != null)
    {
      addRequirements(deflectorSubsystem);
    }

    if( m_bUseTimer == true )
      timer = new Timer();

    var table = NetworkTableInstance.getDefault().getTable("DrivetrainData");
    currentX = table.getEntry("currentX");
    currentY = table.getEntry("currentY");
    
    var shooterTable = NetworkTableInstance.getDefault().getTable("AutomaticShooter");
    tableTargetRPM = shooterTable.getEntry("targetRPM");
    tableMeasuredRPM = shooterTable.getEntry("measuredRPM");
    tableErrorRPM = shooterTable.getEntry("errorRPM");
    tableTemperature = shooterTable.getEntry("temperature");
    tableCurrent = shooterTable.getEntry("current");
    tableIsAtTargetRPM = shooterTable.getEntry("isAtTargetRPM");
    tableTargetDistance = shooterTable.getEntry("targetDistance (m)");
    tableTargetV = shooterTable.getEntry("targetV (m per s)");
    
    var visionTable = NetworkTableInstance.getDefault().getTable(Config.VISION_TABLE_NAME_HUB);
    visionDistance = visionTable.getEntry(Config.DISTANCE_HUB);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if ( m_bUseTimer == true )
    {
      timer.start();
      timer.reset();
    }

    //Calculate the RPM of the shooter wheel.
    calculateTargetRPM();

    //Check if shooter's deflector will be used
    if(deflectorSubsystem != null)
    {
      if(m_bDeflector == true)
      {
        deflectorSubsystem.moveUp();
      }
      else
      {
        deflectorSubsystem.moveDown();
      }
    }
    else
    {
      m_bDeflector = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(shooterSubsystem != null)
    {
      shooterSubsystem.setTargetRPM(targetRPM);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    //print debug info here
    tableTargetRPM.setNumber(targetRPM);

    if ( shooterSubsystem != null )
    {
      tableMeasuredRPM.setNumber(shooterSubsystem.getMeasuredRPM());
      tableErrorRPM.setNumber(targetRPM-shooterSubsystem.getMeasuredRPM());
      tableTemperature.setNumber(shooterSubsystem.getTemperature());
      tableCurrent.setNumber(shooterSubsystem.getCurrentDraw());
      tableIsAtTargetRPM.setBoolean(shooterSubsystem.isAtTargetRPM());

      //stop the shooter
      //shooterSubsystem.setTargetRPM(0); 
      shooterSubsystem.stop();  
    }

    if( m_bUseTimer == true)
      timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( m_bUseTimer == true )
      return timer.get() > timeout;
    else
      return false;
  }

  public void calculateTargetRPM()
  {
    getDistance();

    tableTargetDistance.setNumber(targetDistance);
    //validate distance: for the right range of shooting
    if(targetDistance == -1)
    {
      //Don't set a targetRPM value (default value is 0)
      targetRPM = 0;
      return;
    }

    //@todo: m_bHighGoal can be determined by the distance
    //@todo: m_bDeflector can be determined by the distance
    //@todo: Use distance to determine if we aim for high goal or low goal and if the deflector is used

    m_bHighGoal = true;
    m_bDeflector = false;

    //option1: calculate the target RPM: formula
    if(m_bHighGoal)
    {
      targetHeight = HIGH_GOAL_HEIGHT_IN_METERS;
    }
    else
    {
      targetHeight = LOW_GOAL_HEIGHT_IN_METERS;
    }

    if(m_bDeflector)
    {
      shooterAngle = SHOOTER_ANGLE_DEG_DEFLECTOR;
    }
    else
    {
      shooterAngle = SHOOTER_ANGLE_DEG;
    }

    double targetV  = initVelocity();
    targetRPM       = (int) velocityToRPM (targetV);  

    tableTargetV.setValue(targetV);

    //validate the targetRPM
    //@todo: hard coded here for now
    targetRPM = 0;

    //option2: map the distance to the target RPM
    //From test relationships between the distance and RPM, obtain a formula that represents said relationship
    //targetRPM = 0; //f(targetDistance, m_bDeflector, m_bHighGoal)
  }

  /**
   * Calculate distance from robot to central hub
   */
  public void getDistance()
  {
    //get the distance between the robot and the center of the hub
    //option1: use odometry
    //option2: use vision

    if(m_bVision == false)
    {
      //Using odometry
      double robotX = (double) currentX.getNumber(0.0);
      double robotY = (double) currentY.getNumber(0.0);

      double squareDistance = (HUB_Y-robotY)*(HUB_Y-robotY)+(HUB_X-robotX)*(HUB_X-robotX);
      targetDistance = Math.sqrt(squareDistance);
    }
    else
    {
      //todo: Use vision and network tables to get targetDistance
      targetDistance = visionDistance.getDouble(-1.0);
      System.out.println("targetDistance from vision: "+targetDistance);
      //@todo: Double check if the target distance is valid or not.
      //Note: Unit is feet, we have to convert to meter

    }

    //@todo: validate the targetDistance
    if(targetDistance > 5 || targetDistance < 0.5)
    {
      targetDistance = -1;
    }
  }

  double initVelocity() {
    double dCheck = Math.tan(shooterAngle)*targetDistance - targetHeight;
    double dTemp;

    //unit: m/s
    double dInitVelocity;
    if (dCheck > 0)
    {
         dTemp = Math.sqrt(HALF_OF_GRAVITY/dCheck);
         dInitVelocity = targetDistance/Math.cos(shooterAngle) * dTemp;

         if((dInitVelocity*Math.sin(shooterAngle)) < Math.sqrt(4*HALF_OF_GRAVITY*targetHeight))
         {
          dInitVelocity = 0.0;
          System.out.println("WARNING! Not suitable for shooting!");  
         }
    }
    else
    {
         dInitVelocity = 0.0;
         System.out.println("WARNING! Not suitable for shooting!");      
    }

    return dInitVelocity;
  }

 // convert velocity to RPM
 // velocity: unit m/s
 // return: unit revolutions per minute
double velocityToRPM( double velocity)
 {     
     double rpm = velocity*CONVERSION_NUMBER/(Math.PI*SHOOTER_WHEEL_RADIUS_IN_CM);
     return rpm;
 }
}
