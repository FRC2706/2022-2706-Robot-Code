// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;
import frc.robot.subsystems.DriveBaseHolder;

public class VisionAutoCargo extends CommandBase {
  /** Creates a new VisionAutoAid. */
  final double clampPID = 0.5;
  final double FORWARD_SPEED = 0.2;
  final int CYCLES_NO_VISON = 20;


  public static final FluidConstant<Double> KP = new FluidConstant<>("VisionDriverAidKP", 0.014)
            .registerToTable(Config.constantsTable);
  public static final FluidConstant<Double> KD = new FluidConstant<>("VisionDriverAidKD", 0.0014)
            .registerToTable(Config.constantsTable);
  double targetYaw;
  Supplier<Double> m_rotateVal;
  Supplier<Double> m_visionDistance;
  PIDController m_pid;

  int noVisionCount; 

  Command intakeDownAndSpin;
   
  public VisionAutoCargo(Supplier<Double> rotateVal, Supplier<Double> visionDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBaseHolder.getInstance());
    m_rotateVal = rotateVal;
    m_visionDistance = visionDistance;
    m_pid = new PIDController(0, 0, 0);

    intakeDownAndSpin = new SequentialCommandGroup(
      new ParallelRaceGroup(new IntakeDown(), new WaitCommand(0.5)),
      new ParallelRaceGroup(new IntakeFloat(), new WaitCommand(0.06)),
      new ParallelCommandGroup(new IndexerOneCargo(), new RunIntakeCargo(true, 0)));
  }
   

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.setPID(KP.get(), 0, KD.get());
    targetYaw = DriveBaseHolder.getInstance().getOdometryHeading().getDegrees();
    noVisionCount = 0;

    intakeDownAndSpin.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = DriveBaseHolder.getInstance().getOdometryHeading().getDegrees();
    double visionYaw = m_rotateVal.get();

    if(visionYaw != -99){
      targetYaw = heading + visionYaw;
    }
    if (m_visionDistance.get() == -99) {
      noVisionCount++;
    } else {
      noVisionCount = 0;
    }

    double rotateValue = m_pid.calculate(heading, targetYaw);
    rotateValue = MathUtil.clamp(rotateValue, -clampPID, clampPID);

    DriveBaseHolder.getInstance().arcadeDrive(FORWARD_SPEED,rotateValue, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveBaseHolder.getInstance().stopMotors();
    intakeDownAndSpin.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noVisionCount > CYCLES_NO_VISON;
  }

}
