// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramseteAuto;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWithTime;
import frc.robot.commands.DrivetrainAlignment;
import frc.robot.commands.LowerArm;
import frc.robot.commands.OuterGoalErrorLoop;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.SpinUpShooterWithTime;
import frc.robot.Robot;


/** Add your docs here. */
public class AutoRoutines {
    public static Command getAutoCommandRapidReact(int selectorOne)
    {
        switch(selectorOne)
        {
            case 0:
                // This is our 'do nothing' selector
                return null;
            case 1:
                //Placeholder
                return null;
            default:
                return null;
        }
    }

    public static Command getAutoCommandBeetle(int selectorOne)
    {
        System.out.println("Auto selectorOne: "+selectorOne);
        switch(selectorOne)
        {
            case 0:
                // This is our 'do nothing' selector
                return null;
            case 1:
                //testing blue option 2
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(Robot.trajectoryBlueO2, "Trajectory-Blue-O2");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlueO2.getInitialPose())),
                    ramsete3);
            case 2:
                //testing option 3: path1
                //todo: Check if ending position is the same spot as the starting position of path 2
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Blue3-O1P1");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete2);

            case 3:
                //testing blue option 3
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(Robot.trajectoryBlueO3, "Trajectory-Blue-O3");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlueO3.getInitialPose())),
                    ramsete4);

            case 4: 
                //option 3 (Two blue paths)
                RamseteCommandMerge ramsete3_1 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Blue3-O1P1");
                RamseteCommandMerge ramsete3_2 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P2, "Trajectory-Blue3-O1P2");
                //Pose2d initalPose = Robot.trajectoryReadO1P1.getInitialPose();
                //System.out.println("x: "+initalPose.getX());
                //System.out.println("y: "+initalPose.getY());
                //System.out.println("angle: "+initalPose.getRotation());
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete3_1,         //new ParallelCommandGroup(ramsete3_1, intakeCommand(3)),
                    new WaitCommand(1), //shoot two cargo, no need for alignment. replaced by the shooter command
                    ramsete3_2,         //new ParallelCommandGroup(ramsete3_2, intakeCommand(3)),
                    new DrivetrainAlignment(false) );
                    //@todo: add a shooter command at the end
            case 5:
                //testing red option 1
                RamseteCommandMerge ramsete6 = new RamseteCommandMerge(Robot.trajectoryRedO1, "Trajectory-Red-O1");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryRedO1.getInitialPose())),
                    ramsete6);
            case 6:
                //testing red option 2
                RamseteCommandMerge ramsete7 = new RamseteCommandMerge(Robot.trajectoryRedO2, "Trajectory-Red-O2");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryRedO2.getInitialPose())),
                    ramsete7);
            case 7:
                //testing blue option 3
                RamseteCommandMerge ramsete8 = new RamseteCommandMerge(Robot.trajectoryRedO3, "Trajectory-Red-O3");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryRedO3.getInitialPose())),
                    ramsete8);
            case 8:
                //testing blue option for human player
                RamseteCommandMerge ramsete4_1 = new RamseteCommandMerge(Robot.trajectoryBlue4OP1, "Trajectory-Blue4-O1P1");
                RamseteCommandMerge ramsete4_2 = new RamseteCommandMerge(Robot.trajectoryBlue4OP2, "Trajectory-Blue4-O1P2");
                RamseteCommandMerge ramsete4_3 = new RamseteCommandMerge(Robot.trajectoryBlue4OP3, "Trajectory-Blue4-O1P3");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue4OP1.getInitialPose())),
                    ramsete4_1,         //new ParallelCommandGroup(ramsete3_1, intakeCommand(3)),
                    new WaitCommand(1), //shoot two cargo, no need for alignment. replaced by the shooter command
                    ramsete4_2,
                    new WaitCommand(1),
                    ramsete4_3);         //new ParallelCommandGroup(ramsete3_2, intakeCommand(3)),


            //case 9:
                  
            case 10:
                //testing blue option for human player
                RamseteCommandMerge ramsete8_1 = new RamseteCommandMerge(Robot.trajectoryBlue8P1, "Trajectory-Blue8-P1");
                RamseteCommandMerge ramsete8_2 = new RamseteCommandMerge(Robot.trajectoryBlue8P2, "Trajectory-Blue8-P2");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue8P1.getInitialPose())),
                    ramsete8_1,         //new ParallelCommandGroup(ramsete3_1, intakeCommand(3)),
                    new WaitCommand(2), //shoot two cargo, no need for alignment. replaced by the shooter command
                    ramsete8_2);         //new ParallelCommandGroup(ramsete3_2, intakeCommand(3)),

            default:
                return null;
        }
    }

    //This is just for test
    public static Command getAutoCommandTest(int selectorOne) {

        switch (selectorOne)
        {
            case 0:
            {
                // This is our 'do nothing' selector
                return null;
            } 
            case 1:
            { 
                //Drive with time
                return new DriveWithTime(0.5, 0.5, 0.5);             
            } 
            case 2:
            {
                //Spin up the shooter
                return new SpinUpShooterWithTime((int) Config.RPM.get(), 7); //.andThen(new DriveWithTime(0.5, 0.5, 0.5));
            }
            case 3: 
            {
                // Directly Tell the talons to go both sides a specific value. (For setting inversions)
                SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter);
                double vel = 1.0;

                return new RunCommand(() -> DriveBaseHolder.getInstance().tankDriveVelocities(vel, vel, feedforward.calculate(vel), feedforward.calculate(vel)));
            } 
            case 4:
            {
                //Test ramsete
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                    new Pose2d(), 
                    new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0))), 
                    Config.trajectoryConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false));

                // Run a example ramsete command
                Command resetOdometry = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory.sample(0).poseMeters), DriveBaseHolder.getInstance());
                return resetOdometry.andThen(new RamseteCommandMerge(trajectory, "R4-SingleTraj"));
            } 
            case 5:
            {

                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                    List.of(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24)), // START POSE
                    new Pose2d(5.2, 1.2, Rotation2d.fromDegrees(0))),  // END POSE
                    VisionPose.getInstance().getTrajConfig(0, 2, false)); // CONFIG

                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(5.2, 1.2, Rotation2d.fromDegrees(0)), // START POSE
                    List.of(new Translation2d(3.5, 1.5)), // WAYPOINT
                    new Pose2d(3.0, 2.1, Rotation2d.fromDegrees(-10)),  // END POSE
                    VisionPose.getInstance().getTrajConfig(0, 0, true)); // CONFIG

                return new SequentialCommandGroup(
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24)))),
                    new InstantCommand(() -> FeederSubsystem.getInstance().setBallsAroundFeeder(0)),
                    //new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.5, 7)),
                  //  new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajectory1, "R5FullR-1")),
                 //   new RamseteCommandMerge(trajectory2, "R5FullR-2").alongWith(new IndexBall()),
                    new OuterGoalErrorLoop(true, 3.0)
                   // new ParallelRaceGroup(new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.7, 8)), new AutoIntakeCommand()
                ); 
            } 
            default:
            {
                return null;
            }
        }
    }

    /**
     * Helper method for constructing trajectories. Gets the final pose of a given trajectory.
     * 
     * RamseteCommandMerge has the same functionality with the getTargetPose() method
     * 
     * @param - A given trajectory or ramsete command merge to find what pose it will end at.
     * @return The end pose.
     */
    private static Pose2d endPose(Trajectory trajectory) {
        return trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
    }
    private static Pose2d endPose(RamseteCommandMerge ramsete) {
        return ramsete.getTargetPose();
    }
    
}
