// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

public class VisionDriverAid extends CommandBase {
  /** Creates a new VisionDriverAid. */
  final double clampPID = 0.5;
  public static final FluidConstant<Double> KP = new FluidConstant<>("VisionDriverAidKP", 0.014)
            .registerToTable(Config.constantsTable);
  public static final FluidConstant<Double> KD = new FluidConstant<>("VisionDriverAidKD", 0.0014)
            .registerToTable(Config.constantsTable);
  double targetYaw;
  Supplier<Double> m_forwardVal;
  Supplier<Double> m_rotateVal;
  PIDController m_pid;
  Command shooter;
  Command feed;
  Command shooterAuto;

  public VisionDriverAid(Joystick driveJoystick, Supplier<Double> rotateVal) {
    m_forwardVal = ()-> sign(Config.removeJoystickDeadband(driveJoystick.getRawAxis(Config.LEFT_CONTROL_STICK_Y)), Config.INVERT_FIRST_AXIS);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBaseHolder.getInstance());
    m_rotateVal = rotateVal;
    m_pid = new PIDController(0, 0, 0);
    shooter = new SpinUpShooterWithTime(2000, 0);
    shooterAuto = new AutomaticShooter(true);
    feed = new WaitCommand(0.3).andThen(new ParallelRaceGroup(new IndexerForShooter(), new WaitCommand(1.5)).andThen(new InstantCommand(shooterAuto :: cancel)));
  }
   

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.schedule();
    m_pid.setPID(KP.get(), 0, KD.get());
    targetYaw = DriveBaseHolder.getInstance().getOdometryHeading().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = DriveBaseHolder.getInstance().getOdometryHeading().getDegrees();
    double visionYaw = m_rotateVal.get();

    if(visionYaw != -99){
      targetYaw = heading + visionYaw;
    }

    double rotateValue = m_pid.calculate(heading, targetYaw);
    rotateValue = MathUtil.clamp(rotateValue, -clampPID, clampPID);
    double xSpeed = m_forwardVal.get();
    DriveBaseHolder.getInstance().arcadeDrive(Math.copySign(xSpeed * xSpeed, xSpeed),rotateValue, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted == false){
      shooterAuto.schedule();
      feed.schedule();        
    }
    shooter.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] velocities = DriveBaseHolder.getInstance().getMeasuredMetersPerSecond();
    boolean notMoving = Math.abs(velocities[0])<0.2 && Math.abs(velocities[1])<0.2;
    if(Math.abs(m_rotateVal.get())<3 && notMoving == true){
      return true;
    }
    return false;
  }

  private static double sign(double number, boolean sign){
    if(sign){
      return -number;
    }
    else{
      return number;
    }
  }
}
