// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import java.util.logging.Logger;
import frc.robot.config.Config;

public class DriveToWaypoint extends CommandBase {

  private final RamseteCommandMerge ramseteCommand;
  private final VisionType visionType;
  private final double endAfterTime;
  private final double endVelocity;
  private final Pose2d visionSpotPose;
  private Pose2d desiredPose;
  private int frequency; //The command will calculate a new trajectory every x cycles 
  private int cyclesToRecalculation;

  // Logging
  private Logger logger = Logger.getLogger("DriveToWaypoint");

  /** 
   * Creates a DriveToWaypoint where the robot will drive through the desired pose, given the location of visionPose.
   * 
   * Angle of vision pose should be as if the robot was to drive through the vision spot forwards (as this is what VisionPose will calculate)
   * If the robot is supposed to drive through backwards, desiredPose should be 180deg + what it would be when driving forwards
   */
  public DriveToWaypoint(RamseteCommandMerge ramseteCommand, VisionType visionType, double endAfterTime, double endVelocity, Pose2d visionSpotPose, Pose2d desiredPose) {

    this.ramseteCommand = ramseteCommand;
    this.visionType = visionType;
    this.endAfterTime = endAfterTime;
    this.endVelocity = endVelocity;
    this.visionSpotPose = visionSpotPose;
    this.desiredPose = desiredPose;

    logger.addHandler(Config.logFileHandler);
    this.frequency = 5;
  }
  
  /** 
   * Creates a DriveToWaypoint where the robot will drive through the vision pose.
   * 
   * Angle of vision pose should be as if the robot was to drive through the vision spot forwards (as this is what VisionPose will calculate)
   * Code will correct if the robot should drive through backwards
   */
  public DriveToWaypoint(RamseteCommandMerge ramseteCommand, VisionType visionType, double endAfterTime, double endVelocity, Pose2d visionSpotPose) {

    // Set desired pose equal to vision pose since we want to drive through visionPose.
    this(ramseteCommand, visionType, endAfterTime, endVelocity, visionSpotPose, visionSpotPose);

    // If the robot should drive through backwards then the desired rotation needs to be 180 deg plus the value return from VisionPose
    if (VisionPose.getInstance().getReversed(visionType)) {
      this.desiredPose = new Pose2d(this.desiredPose.getTranslation(), this.desiredPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionPose.getInstance().initVision(visionType);
    cyclesToRecalculation = frequency - 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VisionPose visionPoseInst = VisionPose.getInstance();
      
    // Ask VisionPose for the pose of the endpoint
    Pose2d endPose2d = visionPoseInst.getTargetPose(visionType);

    if(cyclesToRecalculation == 0){
      cyclesToRecalculation = frequency - 1;

      // If the translation isn't null and
      // If the endPose is within a radius of the target we want, 
      // generate the trajectory
      if(endPose2d != null && isAtWaypoint(visionSpotPose, endPose2d, Config.ALLOWABLE_VISION_ODOMETRY_ERROR)){

        // Offset the endPose by the delta between visionPose and desiredPose
        Translation2d deltaTranslation = desiredPose.getTranslation().minus(visionSpotPose.getTranslation());
        Rotation2d deltaRotation = desiredPose.getRotation().rotateBy(visionSpotPose.getRotation().unaryMinus());
        endPose2d = new Pose2d(endPose2d.getTranslation().plus(deltaTranslation), endPose2d.getRotation().rotateBy(deltaRotation));

        // Check whether to use gyro to handle perpendicular angle
        if (Config.useVisionPerpendicularAngle == false) {
          // Overwrite the calculated rotation to what it should be on field oriented dimensions. Reliant on gyro.
          endPose2d = new Pose2d(endPose2d.getTranslation(), desiredPose.getRotation());
        }

        Trajectory trajectory;
        try {
          //Get current robot velocities for left and right sides and find the average
          double[] measuredVelocities = DriveBaseHolder.getInstance().getMeasuredMetersPerSecond();
          double averageCurrentVelocity = (measuredVelocities[0] + measuredVelocities[1])/2.0;

          //Generate trajectory from current pose to endPose
          trajectory = TrajectoryGenerator.generateTrajectory(List.of(DriveBaseHolder.getInstance().getPose(), endPose2d),
                                                              visionPoseInst.getTrajConfig(averageCurrentVelocity, endVelocity, visionType));

        } catch (Exception e) {
          logger.severe("Trajectory Generator failed to calculate a valid trajectory: " + e.getMessage());
          return;
        }

        if(trajectory != null){
          //Give the ramsete command the updated trajectory
          ramseteCommand.setNewTrajectory(trajectory);
          System.out.println("Endpoint Pose is " + endPose2d.toString());
        }

      }
    }
    else{
      cyclesToRecalculation--;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If ramseteCommand isn't scheduled, stop running the command
    if(!ramseteCommand.isScheduled()){
      return true;
    }
    return false;
  }

  //Set recaclulation frequency to new value
  public void setFrequency(int frequency){
    this.frequency = frequency;
  }

  //Calculates whether the robot is within a certain radius of the waypoint
  private boolean isAtWaypoint(Pose2d waypointPose, Pose2d currentPose, double radiusMeters){
    if(waypointPose != null){
        //Find X and Y distance between waypointPose and currentPose
      double deltaX = currentPose.getX() - waypointPose.getX();
      double deltaY = currentPose.getY() - waypointPose.getY();

      //If distance is less than radiusMeters, return true
      if(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)) <= radiusMeters){
        return true;
      }
    }
    return false;
  }
}
