// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShooter extends CommandBase {
  private ShooterSubsystem shooterSubsystem;

  //calculated for the shooter
  double targetDistance = 0;
  int targetRPM = 0;

  //@todo: final tuning
  private final int RPM_TOLERANCE = 75;
  private Timer timer;


  private int timeout;

  //todo: can be configured in config file as well
  //todo: measure the radius for the shooting wheel
  private final double SHOOTER_ANGLE_IN_DEGREES  = 60.0;
  private final double TARGET_HEIGHT_IN_METERS = 2.49;
  private final double SHOOTER_WHEEL_RADIUS_IN_CM = 10;
  private final double HALF_OF_GRAVITY = 4.91;
  private final double CONVERSION_NUMBER = 3000;

  /** Creates a new AutomaticShooter. */
  public AutomaticShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = ShooterSubsystem.getInstance();
    if (shooterSubsystem != null) 
    {
      addRequirements(shooterSubsystem);
    }

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer.start();
    //Calculate the RPM of the shooter wheel.
    double targetV  = initVelocity(targetDistance);
    targetRPM     = (int) velocityToRPM (targetV);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if( shooterSubsystem != null )
    {
        shooterSubsystem.setTargetRPM(targetRPM);
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    //print debug info here
    SmartDashboard.putNumber("shooter measured RPM", shooterSubsystem.getRPM());
    SmartDashboard.putNumber("shooter target RPM", targetRPM);
    SmartDashboard.putNumber("shooter error RPM", targetRPM-shooterSubsystem.getRPM());
    SmartDashboard.putNumber("shooter temp", shooterSubsystem.getTemperature());
    SmartDashboard.putNumber("shooter current", shooterSubsystem.getCurrentDraw());
    SmartDashboard.putBoolean("shooter isTargetRPM", shooterSubsystem.isAtTargetRPM());
          
    //stop the shooter
    shooterSubsystem.setTargetRPM(0);   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > timeout;
  }

  public void calculateTargetRPM()
  {
    //get the distance between the robot and the center of the hub
    //option1: use odometry

    //option2: use vision

    //validate the distance for shooting


    //option1: calculate the target RPM: formula


    //option2: map the distance to the target RPM

  

  }

  double initVelocity(double distanceToTargetInMeters) {
    double dCheck = Math.tan(SHOOTER_ANGLE_IN_DEGREES)*distanceToTargetInMeters - TARGET_HEIGHT_IN_METERS;
    double dTemp;

    //unit: m/s
    double dInitVelocity;
    if (dCheck > 0)
    {
         dTemp = Math.sqrt(HALF_OF_GRAVITY/dCheck);
         dInitVelocity = distanceToTargetInMeters/Math.cos(SHOOTER_ANGLE_IN_DEGREES) * dTemp;
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
