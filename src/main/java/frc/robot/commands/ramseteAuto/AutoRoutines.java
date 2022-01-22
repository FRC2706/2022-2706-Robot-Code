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
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.DriveWithTime;
import frc.robot.commands.IndexBall;
import frc.robot.commands.LowerArm;
import frc.robot.commands.OuterGoalErrorLoop;
import frc.robot.commands.RunFeederCommandWithTime;
import frc.robot.commands.SpinUpShooterWithTime;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.subsystems.FeederSubsystem;

/** Add your docs here. */
public class AutoRoutines {
    public static Command getAutoCommandTest(int selectorOne) {

        if (selectorOne == 0) {
            // This is our 'do nothing' selector
            return null;
        } else if (selectorOne == 1) { 
            return new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.7, 7)); //.andThen(new DriveWithTime(0.5, 0.5, 0.5));
           // return new DriveWithTime(AUTO_DRIVE_TIME,  AUTO_LEFT_MOTOR_SPEED,  AUTO_RIGHT_MOTOR_SPEED);
        
        } else if(selectorOne == 2) {
            return new DriveWithTime(0.5, 0.5, 0.5); 

        } else if(selectorOne == 3) {
            // Directly Tell the talons to go both sides a specific value. (For setting inversions)
            SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter);
            double vel = 1.0;

            return new RunCommand(() -> DriveBaseHolder.getInstance().tankDriveVelocities(vel, vel, feedforward.calculate(vel), feedforward.calculate(vel)));

        
            /** 
             * TESTING RAMSETE PATH
             * 
             */
        } else if(selectorOne == 4) {
            
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(), 
                new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0))), 
                Config.trajectoryConfig.setStartVelocity(0).setEndVelocity(0).setReversed(false));

            // Run a example ramsete command
            Command resetOdometry = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory.sample(0).poseMeters), DriveBaseHolder.getInstance());
            

            return resetOdometry.andThen(new RamseteCommandMerge(trajectory, "R4-SingleTraj"));

        } else if (selectorOne == 5) {

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
                new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.5, 7)),
                new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajectory1, "R5FullR-1")),
                new RamseteCommandMerge(trajectory2, "R5FullR-2").alongWith(new IndexBall()),
                new OuterGoalErrorLoop(true, 3.0),
                new ParallelRaceGroup(new SpinUpShooterWithTime((int) Config.RPM.get(), 7).alongWith(new RunFeederCommandWithTime(-0.7, 8)), new AutoIntakeCommand())
            ); 



        } else if (selectorOne == 6) {
            // Run a example ramsete command
            Command resetOdometry = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d()), DriveBaseHolder.getInstance());

            Trajectory trajDriveForward = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(), 
                new Pose2d(1.5, 0, new Rotation2d(0))), 
                VisionPose.getInstance().getTrajConfig(0, 0, false));

            ParallelRaceGroup cmdGroup = new ParallelRaceGroup(new AutoIntakeCommand(), new RamseteCommandMerge(trajDriveForward, "R6DriveForward"));
            return resetOdometry.andThen(cmdGroup);

        } else if (selectorOne == 7) {
            return new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d(2.8, 1.7, Rotation2d.fromDegrees(-24))));
            // return new OuterGoalErrorLoop(true, 3.0);

        } else if (selectorOne == 8) {
            return new InstantCommand(() -> FeederSubsystem.getInstance().setBallsAroundFeeder(0));
        } else if (selectorOne == 9) {

            Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // A
                new Pose2d(1.7, 0, Rotation2d.fromDegrees(0))), // B
                VisionPose.getInstance().getTrajConfig(0, 0, false)
            );

            Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(1.7, 0, Rotation2d.fromDegrees(0)), // B
                new Pose2d(0.5, -0.72, Rotation2d.fromDegrees(90)), // B'
                new Pose2d(1.29, -1.3, Rotation2d.fromDegrees(171))),  // C
                VisionPose.getInstance().getTrajConfig(0, 0, true)
            );

            Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(1.29, -1.3, Rotation2d.fromDegrees(171)), // C
                new Pose2d(0.0, -0.3, Rotation2d.fromDegrees(180)), // C'
                new Pose2d(-1.5, 0.05, Rotation2d.fromDegrees(160))), // D
                new TrajectoryConfig(Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Config.kDriveKinematics).addConstraint(Config.autoVoltageConstraint)
                    .addConstraint(new RectangularRegionConstraint(new Translation2d(-3, 0.5), new Translation2d(-0.5, -2.0), new MaxVelocityConstraint(0.8)))   // VisionPose.getInstance().getTrajConfig(0, 0, false)
            );

            Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(-1.5, 0.05, Rotation2d.fromDegrees(160)), // D
                new Pose2d(-0.62, -0.16, Rotation2d.fromDegrees(-150))), // E
                VisionPose.getInstance().getTrajConfig(0, 0, true)
            );
            
            Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(-0.62, -0.16, Rotation2d.fromDegrees(-150)),
                new Pose2d(-1.48, -0.38, Rotation2d.fromDegrees(-160))),
                VisionPose.getInstance().getTrajConfig(0, 0, false)
            );

            Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(-1.48, -0.38, Rotation2d.fromDegrees(-160)),
                new Pose2d(-0.34, -0.52, Rotation2d.fromDegrees(150))),
                VisionPose.getInstance().getTrajConfig(0, 0, true)
            );

            return new SequentialCommandGroup(
                new InstantCommand(DriveBaseHolder.getInstance()::setBrakeMode),
                new ParallelRaceGroup(new RamseteCommandMerge(trajectory1, "IRAHr2-T1"), new AutoIntakeCommand()),
                new RamseteCommandMerge(trajectory2, "IRAHr2-T2"),
                new OuterGoalErrorLoop(true, 3.0),
                new SpinUpShooterWithTime((int) Config.RPM.get()+300, 7).alongWith(new RunFeederCommandWithTime(-0.5, 7)),
                new ParallelRaceGroup(new RamseteCommandMerge(trajectory3, "IRAHr3-T2"), new AutoIntakeCommand()),
                new RamseteCommandMerge(trajectory4, "IRAHr4-T2"),
                new ParallelRaceGroup(new RamseteCommandMerge(trajectory5, "IRAHr5-T2"), new AutoIntakeCommand()),
                new RamseteCommandMerge(trajectory6, "IRAHr6-T2"),

                new OuterGoalErrorLoop(true, 3.0),
                new SpinUpShooterWithTime((int) Config.RPM.get()+700, 7).alongWith(new RunFeederCommandWithTime(-0.5, 7))
            );

        } else if (selectorOne == 10) {
            Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(), 
                List.of(new Translation2d(1, -0.4)),
                new Pose2d(1.5, -1.7, Rotation2d.fromDegrees(-45)),
                VisionPose.getInstance().getTrajConfig(0, 0, VisionPose.VisionType.TPracticeTarget));

            RamseteCommandMerge ramsete = new RamseteCommandMerge(trajectory1, "PassThruWaypointTest");

            // return ramsete;
            return new ParallelCommandGroup(ramsete,
                    // new PassThroughWaypoint(ramsete, endPose(trajectory1), VisionPose.VisionType.TPracticeTarget, 8, 0, 0.5),
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(new Pose2d()))); 
        }


        // Also return null if this ever gets to here because safety
        return null;
    }

    public static Command getAutoCommandIRAH(int selectorOne) {
        switch (selectorOne) {

            case 0:
                return null;

            case 1:{
                /** Bounce path  */
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    new PoseScaled(0.9, -2.4, 0+180),
                    new PoseScaled(2.25, -1, 90+180)),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTurnAroundSpeed, true));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Bounce-P1");

                Pose2d middleOfConesD3toD5 = new PoseScaled(3.0, -3.1, 90+180);
                Pose2d desiredMiddleOfConesD3toD5 = new PoseScaled(3.0, -3.1, 90+25+180);
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory1),
                    desiredMiddleOfConesD3toD5,
                    new PoseScaled(3.9, -3.95, 180+180)),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Bounce-P2");

                Pose2d middleOfConesD5toD7 = new PoseScaled(4.6, -3.1, -90+180);
                Pose2d middleOfConesB5toB7 = new PoseScaled(4.58, -1.56, -90+180);
                Pose2d desiredMiddleOfConesB5toB7 = new PoseScaled(4.58, -1.11, -90+180);
                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory2),
                    middleOfConesD5toD7,
                    desiredMiddleOfConesB5toB7),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTurnAroundSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Bounce-P3");

                PoseScaled bounceFirstDiamondMarkerA9 = new PoseScaled(6.872, -1, 90);
                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory3), 
                    middleOfConesD5toD7, 
                    new PoseScaled(5.75, -3.8, 0), 
                    new PoseScaled(6.86, -3.1 , 90),
                    bounceFirstDiamondMarkerA9), 
                    VisionPose.getInstance().getTrajConfig(0, 0, false));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Bounce-P4");

                Pose2d middleOfConesB10toD10 = new PoseScaled(7.58, -2.28, 180+180);
                Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(
                    endPose(trajectory4), List.of(
                    new TranslationScaled(7.16, -1.9),
                    middleOfConesB10toD10.getTranslation()),
                    new PoseScaled(8.37, -2.34, 180+180),
                    VisionPose.getInstance().getTrajConfig(0, 0, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Bounce-P5");

                double waypointRadiusMeters = 0.5;

                return new SequentialCommandGroup(
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                    ramsete1,
                    new ParallelRaceGroup(ramsete2, new PassThroughWaypoint(ramsete2, endPose(ramsete2), middleOfConesD3toD5, desiredMiddleOfConesD3toD5, VisionType.MiddleOfCones, 6, Config.kRamseteTransferSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete3, new PassThroughWaypoint(ramsete3, endPose(ramsete3), middleOfConesB5toB7, desiredMiddleOfConesB5toB7, VisionType.MiddleOfCones, 6, Config.kRamseteTurnAroundSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete4, new DriveToWaypoint(ramsete4, VisionType.DiamondTape, 10, Config.kRamseteTurnAroundSpeed, bounceFirstDiamondMarkerA9, new PoseScaled(6.872, -1, 90))),
                    new ParallelRaceGroup(ramsete5, new PassThroughWaypoint(ramsete5, endPose(ramsete4), middleOfConesB10toD10, VisionType.MiddleOfCones, 6, 0, waypointRadiusMeters))
                );
            }
            case 2:{
                // Barrel Racing path

              
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(new PoseScaled (0.0,0.0,180+0.0), 
                  List.of(
                    new TranslationScaled(1.026, 0.20),
                    new TranslationScaled(1.763, -0.4),
                    new TranslationScaled(1.45,-0.808),
                    new TranslationScaled(1.1,-0.302)), //<--- need to be adjusted. also add angles
                    new PoseScaled(1.179,0.1,180+0),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Barrel-P1");
                //Config.kRamseteTransferSpeed,
                //VisionType.DiamondTape
                
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of( 
                    endPose(trajectory1),
                    new PoseScaled(2.2, -0.134, 180+10), 
                    new PoseScaled(3.29,0.226, 180+48)), 
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Barrel-P2");
                //Config.kRamseteTransferSpeed
                //VisionType.DiamondTape


                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory( 
                  List.of(
                    endPose(trajectory2),
                    new PoseScaled(3.257,0.99,180+148),
                    new PoseScaled(2.297,1.019,180-141),
                    new PoseScaled(2.165,0.101,180-54.79),
                    new PoseScaled(3.182,-0.818,180+0)),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Barrel-P3");
                //VisionType.DiamondTape

                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
                    List.of( 
                    endPose(trajectory3),
                    new PoseScaled( 4.21,-0.529,180+94), 
                    new PoseScaled( 3.89,0.05,180+147), 
                    new PoseScaled(3.128,-0.15,180-163),
                    new PoseScaled(2.039,0.10,180+180),
                    new PoseScaled(0.0,0.0,180+180)),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, 0, true));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Barrel-P4");

                // Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                //   List.of(
                //     new TranslationScaled(),
                //     new TranslationScaled(),
                //     new TranslationScaled()),
                //     new PoseScaled(),
                //     VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                // RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Barrel-P5");
            
                return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                  ramsete1,
                                                  ramsete2,
                                                  ramsete3,
                                                  ramsete4);
             }
            
        }

        // If nothing runs do nothing
        return null;   

    }

    public static Command getAutoCommandIRAHDeepSpaceRobot(int selectorOne) {
        switch (selectorOne) {

            case 0:
                return null;

            case 1:{
                /** Bounce path  */
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    new PoseScaled(0.9, -2.4, 0),
                    new PoseScaled(2.25, -1, 90)),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTurnAroundSpeed, true));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Bounce-P1");

                Pose2d middleOfConesD3toD5 = new PoseScaled(3.0, -3.1, 90);
                Pose2d desiredMiddleOfConesD3toD5 = new PoseScaled(3.0, -3.1, 90+25);
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory1),
                    desiredMiddleOfConesD3toD5,
                    new PoseScaled(3.9, -3.95, 180)),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Bounce-P2");

                Pose2d middleOfConesD5toD7 = new PoseScaled(4.6, -3.1, -90);
                Pose2d middleOfConesB5toB7 = new PoseScaled(4.58, -1.56, -90);
                Pose2d desiredMiddleOfConesB5toB7 = new PoseScaled(4.58, -1.11, -90);
                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory2),
                    middleOfConesD5toD7,
                    desiredMiddleOfConesB5toB7),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTurnAroundSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Bounce-P3");

                PoseScaled bounceFirstDiamondMarkerA9 = new PoseScaled(6.872, -1, 90);
                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory3), 
                    middleOfConesD5toD7, 
                    new PoseScaled(5.75, -3.8, 0), 
                    new PoseScaled(6.86, -3.1 , 90),
                    bounceFirstDiamondMarkerA9), 
                    VisionPose.getInstance().getTrajConfig(0, 0, VisionType.DiamondTape));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Bounce-P4");

                Pose2d middleOfConesB10toD10 = new PoseScaled(7.58, -2.28, 180);
                Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(
                    endPose(trajectory4), List.of(
                    new TranslationScaled(7.16, -1.9),
                    middleOfConesB10toD10.getTranslation()),
                    new PoseScaled(8.37, -2.34, 180),
                    VisionPose.getInstance().getTrajConfig(0, 0, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Bounce-P5");

                double waypointRadiusMeters = 0.5;

                return new SequentialCommandGroup(
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                    ramsete1,
                    new ParallelRaceGroup(ramsete2, new PassThroughWaypoint(ramsete2, endPose(ramsete2), middleOfConesD3toD5, desiredMiddleOfConesD3toD5, VisionType.MiddleOfCones, 6, Config.kRamseteTransferSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete3, new PassThroughWaypoint(ramsete3, endPose(ramsete3), middleOfConesB5toB7, desiredMiddleOfConesB5toB7, VisionType.MiddleOfCones, 6, Config.kRamseteTurnAroundSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete4, new DriveToWaypoint(ramsete4, VisionType.DiamondTape, 10, Config.kRamseteTurnAroundSpeed, bounceFirstDiamondMarkerA9, new PoseScaled(6.872, -1, 90))),
                    new ParallelRaceGroup(ramsete5, new PassThroughWaypoint(ramsete5, endPose(ramsete4), middleOfConesB10toD10, VisionType.MiddleOfCones, 6, 0, waypointRadiusMeters))
                );
            }
            case 2:{
                // Barrel Racing path

              
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(new PoseScaled (0.0,0.0,180+0.0), 
                  List.of(
                    new TranslationScaled(1.026, 0.20),
                    new TranslationScaled(1.763, -0.4),
                    new TranslationScaled(1.45,-0.808),
                    new TranslationScaled(1.1,-0.302)), //<--- need to be adjusted. also add angles
                    new PoseScaled(1.179,0.1,180+0),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Barrel-P1");
                //Config.kRamseteTransferSpeed,
                //VisionType.DiamondTape
                
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of( 
                    endPose(trajectory1),
                    new PoseScaled(2.2, -0.134, 180+10), 
                    new PoseScaled(3.29,0.226, 180+48)), 
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Barrel-P2");
                //Config.kRamseteTransferSpeed
                //VisionType.DiamondTape


                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory( 
                  List.of(
                    endPose(trajectory2),
                    new PoseScaled(3.257,0.99,180+148),
                    new PoseScaled(2.297,1.019,180-141),
                    new PoseScaled(2.165,0.101,180-54.79),
                    new PoseScaled(3.182,-0.818,180+0)),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Barrel-P3");
                //VisionType.DiamondTape

                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
                    List.of( 
                    endPose(trajectory3),
                    new PoseScaled( 4.21,-0.529,180+94), 
                    new PoseScaled( 3.89,0.05,180+147), 
                    new PoseScaled(3.128,-0.15,180-163),
                    new PoseScaled(2.039,0.10,180+180),
                    new PoseScaled(0.0,0.0,180+180)),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, 0, true));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Barrel-P4");

                // Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                //   List.of(
                //     new TranslationScaled(),
                //     new TranslationScaled(),
                //     new TranslationScaled()),
                //     new PoseScaled(),
                //     VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                // RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Barrel-P5");
            
                return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                  ramsete1,
                                                  ramsete2,
                                                  ramsete3,
                                                  ramsete4);
             }

            case 3: {

                // Church parking lot -> barrel racing
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    new PoseScaled(-0.2, 0, 0.0),
                    new PoseScaled(3.045, -0.143, -15.908),
                    new PoseScaled(3.5, -1.108, -102.612),
                    new PoseScaled(2.6, -1.75, 178.770),
                    new PoseScaled(2.0, -0.800, 81.606),
                    new PoseScaled(2.968, 0.3, 7.646),
                    new PoseScaled(5.503, 0.329, 22.544),
                    new PoseScaled(5.970, 1.45, 117.070),
                    new PoseScaled(4.627, 1.45, -137.681),
                    new PoseScaled(4.471, 0.460, -55.1),
                    new PoseScaled(6.748, -1.48, -4.8),
                    new PoseScaled(7.628, -0.456, 98.6),
                    new PoseScaled(6.869, 0.25, 172.7),
                    new PoseScaled(4.191, 0.017, 179.78),
                    new PoseScaled(-0.5, 0.8, -180)),
                    VisionPose.getInstance().getTrajConfig(0, 0, false));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-DSRobot-BarrelRacing-P1");
                return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                    new LowerArm(),
                                                    ramsete1);
                //                                   ramsete2,
                //                                   ramsete3,
                //                                   ramsete4);
            }
            
        }

        // If nothing runs do nothing
        return null;   

    }

    public static Command getAutoCommandIRAHPracBot(int selectorOne) {
        switch (selectorOne) {
            case 0:
                return null;

            case 1: {
                // Bounce
                double turnAroundSpeed = Config.kMaxSpeedMetersPerSecond;

                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    new PoseScaled(0.3, 0.0, 0),
                    new PoseScaled(1.4, 1.45, 100)),
                    VisionPose.getInstance().getTrajConfig(0, 0, false));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAHPrac-Bounce-P1");

                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory1),
                    new PoseScaled(2.55, -0.549, 90),
                    new PoseScaled(3.026, -1.5, 179.648),
                    new PoseScaled(3.65, -0.188, -89.385),
                    new PoseScaled(3.819, 1.417, -90.176)),
                    VisionPose.getInstance().getTrajConfig(0, turnAroundSpeed, true));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAHPrac-Bounce-P2");

                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory2), 
                    new PoseScaled(3.9, -0.190, -90.791),
                    new PoseScaled(5.014, -1.555, -3.560),
                    new PoseScaled(5.971, 0.125, 90.835),
                    new PoseScaled(6.2, 1.223, 88.989)),
                    VisionPose.getInstance().getTrajConfig(0, 0, false));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAHPrac-Bounce-P3");

                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory3),
                    new PoseScaled(6.371, 0.772, 117.334),
                    new PoseScaled(7.050, 0.2, 143.877)),
                    VisionPose.getInstance().getTrajConfig(0, 0, true));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAHPrac-Bounce-P3");


//List.of(new TranslationScaled(6.2, 0.5)),
// new PoseScaled(7.15, -0.1, 160),
                return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                new LowerArm(),
                                                ramsete1,
                                                ramsete2,
                                                ramsete3,
                                                ramsete4); 
                }
            default:
                return null;


//  new PoseScaled(2.230, -0.549, 117.422) 
//  new PoseScaled(3.026, -1.396, 179.648) 
//  new PoseScaled(3.826, -0.188, -89.385) 
//  new PoseScaled(3.819, 1.417, -90.176) 

//  new PoseScaled(3.792, -0.190, -90.791) 
//  new PoseScaled(5.014, -1.555, -3.560) 
//  new PoseScaled(5.971, 0.125, 90.835) 
//  new PoseScaled(5.991, 1.223, 88.989) 

//  new PoseScaled(7.078, -0.042, 176.396) 
//  new PoseScaled(7.078, -0.042, 175.869)       
        }
    }

    public static Command getAutoCommandIRAHMiniBot(int selectorOne){
        switch (selectorOne) {
            case 0:
                return null;
            case 2:
            {
                // Barrel Racing path tested in basement
                // robot: backward driving
              
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(new PoseScaled (0.0,0.0,180+0.0), 
                  List.of(
                    new TranslationScaled(1.026, 0.20),
                    new TranslationScaled(1.763, -0.4),
                    new TranslationScaled(1.45,-0.808),
                    new TranslationScaled(1.1,-0.302)), //<--- need to be adjusted. 
                    new PoseScaled(1.179,0.1,180+0),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, true));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Barrel-P1");
                    //Config.kRamseteTransferSpeed,
                    //VisionType.DiamondTape
                    
                    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of( 
                        endPose(trajectory1),
                        new PoseScaled(2.2, -0.134, 180+10), 
                        new PoseScaled(3.29,0.226, 180+48)), 
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                    RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Barrel-P2");
                    //Config.kRamseteTransferSpeed
                    //VisionType.DiamondTape
    
    
                    Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory( 
                      List.of(
                        endPose(trajectory2),
                        new PoseScaled(3.257,0.99,180+148),
                        new PoseScaled(2.297,1.019,180-141),
                        new PoseScaled(2.165,0.101,180-54.79),
                        new PoseScaled(3.182,-0.818,180+0)),
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                    RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Barrel-P3");
                    //VisionType.DiamondTape
    
                    Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
                        List.of( 
                        endPose(trajectory3),
                        new PoseScaled( 4.21,-0.529,180+94), 
                        new PoseScaled( 3.89,0.05,180+147), 
                        new PoseScaled(3.128,-0.15,180-163),
                        new PoseScaled(2.039,0.10,180+180),
                        new PoseScaled(0.0,0.0,180+180)),
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, 0, true));
                    RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Barrel-P4");
    
                    // Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                    //   List.of(
                    //     new TranslationScaled(),
                    //     new TranslationScaled(),
                    //     new TranslationScaled()),
                    //     new PoseScaled(),
                    //     VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                    // RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Barrel-P5");
                
                    return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                      ramsete1,
                                                      ramsete2,
                                                      ramsete3,
                                                      ramsete4);
            }
            case 3:
            {
                //slalom path
                //Not completed on trajectory 4
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                    List.of(
                       new PoseScaled (0.0,0.0,0.0), 
                       new PoseScaled(0.773, 0.241, 83),
                       new PoseScaled(1.162,1.101,30.93)),
                        // new PoseScaled(2.0,1.14,0),
                        //    new PoseScaled(1.96,1.46,1.8)),
                       VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, false));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Slalom-P1");
    
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                        List.of( 
                        endPose(trajectory1),
                        new PoseScaled(2.4,1.13,-63),
                        new PoseScaled(3.54,-0.0577,-5),
                        new PoseScaled(4.23,1.26,95),
                        new PoseScaled(3.8,1.36,179)), 
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, false));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Slalom-P2");
    
                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of( 
                        endPose(trajectory2),
                        new PoseScaled(3.34,1.06,-103),
                        new PoseScaled(3.0,-0.03,-130.5)),
                    //    new PoseScaled(2.4,0.03,-146)), 
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, false));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Slalom-P3");
    
                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of( 
                        endPose(trajectory3),
                        
                        new PoseScaled(1.23,-0.16,145),
                        new PoseScaled(0.87,0.78,-172),
                        new PoseScaled(0,0,-180)), 
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, 0, false));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Slalom-P4");
                   
                return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                      ramsete1,                             
                                                      ramsete2,
                                                      ramsete3, 
                                                      ramsete4);    
    
            }
            case 4:
            {
                    //Added Alex's slalom path.
                    //Based on pathweaver, not tested
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(new PoseScaled(1.284, -3.93, 0.0), 
                    List.of(
                      new TranslationScaled(2.147, -3.576) ),
                      new PoseScaled(3.389,-2.03, 21),
                      VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTransferSpeed, false));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Slolam-P1");
    
                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                    List.of(
                      new TranslationScaled(4.693, -1.776),
                      new TranslationScaled(5.88, -1.972),
                      new TranslationScaled(6.514, -2.718),
                      new TranslationScaled(7.451, -3.73),
                      new TranslationScaled(8.518, -3.643),
                      new TranslationScaled(8.28 , -2.234),
                      new TranslationScaled(6.976, -2.559)),
                      new PoseScaled(6.105, -3.847, 204.2),
                      VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, 0, false));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Slolam-P3");
    
                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                    List.of(
                    new TranslationScaled(4.68, -4.009),
                    new TranslationScaled(3.147, -3.847),
                    new TranslationScaled(2.564, -3.368)),
                    new PoseScaled(1.609, -2.289, 151),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, VisionType.MiddleOfCones));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Slolam-P3");
    
                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(new PoseScaled(), 
                    List.of(),
                      new PoseScaled(0.593, -2.143, 0),
                      VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, 0, true));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Slolam-P4");
    
                return new SequentialCommandGroup(new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                                                      ramsete1,
                                                      ramsete2                             
                                                     );   
            } 

            case 5: 
            {
                 //manually tuned on mini-robot in basement
                 //Bounce path
                Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    //new PoseScaled(0.372, 0.058, 37.57),
                    new PoseScaled(0, 0, 0),
                    new PoseScaled(0.622, 0.36, 75.32)),
                    VisionPose.getInstance().getTrajConfig(0, Config.kRamseteTurnAroundSpeed, false));
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAH-Bounce-P1");

                Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory1),
                    new PoseScaled(1.34, -0.609, 83.35)), // MATCH-VisionPose First MiddleOfCones 
                    //new PoseScaled(3.9, -3.95, 180)),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "IRAH-Bounce-P2");

                Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory2),
                    new PoseScaled(1.73, 0.5, 56.82)), // MATCH-Trajectory4
                    //new PoseScaled(4.58, -1.11, -90)),  // MATCH-VisionPose
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, false));
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "IRAH-Bounce-P3");

                //VisionType.MiddleOfCones

                Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory3), 
                    new PoseScaled(2.103, -0.454, 93.65)), // MATCH-Trajectory3
                    //new PoseScaled(5.75, -3.8, 0), 
                    //new PoseScaled(6.86, -3.1 , 90),
                    //new PoseScaled(6.872, -1, 90)), // MATCH-VisionPose First Diamond for third starred marker
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, true));
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(trajectory4, "IRAH-Bounce-P4");

                //VisionType.DiamondTape

                Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory4), 
                    new PoseScaled(2.73, 0.5, 69.61)),
                    //new TranslationScaled(7.58, -2.28)), // MATCH-VisionPose End zone Middle of Cones (B10 to D10)
                    //new PoseScaled(8.37, -2.34, 180),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteTransferSpeed, Config.kRamseteTransferSpeed, false));
                RamseteCommandMerge ramsete5 = new RamseteCommandMerge(trajectory5, "IRAH-Bounce-P5");

                //VisionType.MiddleOfCones

                Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory5), 
                    new PoseScaled(2.3, -0.1, -10)),
                    //new TranslationScaled(7.58, -2.28)), // MATCH-VisionPose End zone Middle of Cones (B10 to D10)
                    //new PoseScaled(8.37, -2.34, 180),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteBounceEndSpeed, Config.kRamseteBounceEndSpeed, true));
                RamseteCommandMerge ramsete6 = new RamseteCommandMerge(trajectory6, "IRAH-Bounce-P6");

                Trajectory trajectory7 = TrajectoryGenerator.generateTrajectory(List.of(
                    endPose(trajectory6), 
                    new PoseScaled(3.46, -0.1, -10)),
                    //new TranslationScaled(7.58, -2.28)), // MATCH-VisionPose End zone Middle of Cones (B10 to D10)
                    //new PoseScaled(8.37, -2.34, 180),
                    VisionPose.getInstance().getTrajConfig(Config.kRamseteBounceEndSpeed, 0, false));
                RamseteCommandMerge ramsete7 = new RamseteCommandMerge(trajectory7, "IRAH-Bounce-P7");

                //VisionType.MiddleOfCones
               
                double waypointRadiusMeters = 0.5;

                return new SequentialCommandGroup(
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                    ramsete1,
                    new ParallelRaceGroup(ramsete2),// new PassThroughWaypoint(ramsete2, VisionType.MiddleOfCones, 6, ramsete2.getTargetPose(), Config.kRamseteTransferSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete3),// new PassThroughWaypoint(ramsete3, VisionType.MiddleOfCones, 6, ramsete3.getTargetPose(), Config.kRamseteTurnAroundSpeed, waypointRadiusMeters)),
                    new ParallelRaceGroup(ramsete4),// new DriveToWaypoint(ramsete4, VisionType.DiamondTape, 10, Config.kRamseteTurnAroundSpeed)),
                    new ParallelRaceGroup(ramsete5),// new PassThroughWaypoint(ramsete5, VisionType.MiddleOfCones, 6, ramsete4.getTargetPose(), 0, waypointRadiusMeters))
                    new ParallelRaceGroup(ramsete6),
                    new ParallelRaceGroup(ramsete7)
                    );
            }
            
        }
        return null;
                
    }

    
    public static Command getAutoCommandIRAHCompetitionBot(int selectorOne){ 
        System.out.println("SelectorOne: "+selectorOne);
        switch (selectorOne) {
            case 0:
                return null;

            case 1:
                {
                    //Galactic: path A
                    //first determine if the red or blue path
                    double dYaw = VisionCtrlNetTable.yawToDiamond.get();
    
                    boolean bRed = false;
                    if ( dYaw > 0 )
                    {
                       bRed = true;
                    }

                    //Hardcoded, will be removed later
                    bRed = false;
                    System.out.println("Path A, bRed: " + bRed);
   
                    //notes:
                    // different speed for picking up the ball
                    // parallel command with picking power cell
                    if ( bRed )
                    {
                        //red path
    
                        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                            new PoseScaled(0, 0, 0),
                            new PoseScaled(1.062, 0.001, 0.220),    //C3 
                            new PoseScaled(3.251, -0.731, -28.521), //D5  (y=-0.631)
                            new PoseScaled(4.127, 1.060, 83.013)),  //A6
                            VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                        RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path A red");
    
                        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                            endPose(trajectory1), //A6
                            new PoseScaled(5.331, 1.608, 1.978),
                            new PoseScaled(8.479, 1.769, 4.702)),
                            VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                        RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path A red");
                        
                        Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                        return new SequentialCommandGroup(
                            new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                            new LowerArm(),
                            new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                        //return new LowerArm();
                        
                    }
                    else
                    {
                        //blue path
                        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                            new PoseScaled(0, 0, 0),
                            new PoseScaled(3.638, -0.059, 0.220), //E6
                            new PoseScaled(4.801, 1.609, 65.391), //B7
                            new PoseScaled(6.154, 1.456, -32.300)), //C9
                            VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                        RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path A blue");
    
                        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                            endPose(trajectory1), //C9
                            new PoseScaled(8.287, 0.542, 0.571)),
                            VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                        RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path A blue");
                        
                        Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                        return new SequentialCommandGroup(
                            new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                            new LowerArm(),
                            new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                        
                    }
                }
            case 2:
            {
                //Galactic: path B
                //first determine if the red or blue path
                double dYaw = VisionCtrlNetTable.yawToDiamond.get();
                boolean bRed = false;
                if ( dYaw > 0 )
                {
                   bRed = true;
                }

                System.out.println("Path B, bRed: "+bRed);
   
                //notes:
                // different speed for picking up the ball
                // parallel command with picking power cell
                if ( bRed )
                {
                    //red path

                    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                        new PoseScaled(0, 0, 0),
                        new PoseScaled(1.614, 0.060, 1.978),   //B3
                        new PoseScaled(3.123, -1.2, -42.539),  //D5
                        new PoseScaled(4.142, -2.309, 6.680),  
                        new PoseScaled(5.3, -0.288, 53.394)),  //B7
                        VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path B red");
    
                    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                        endPose(trajectory1), //B7 
                        new PoseScaled(6.034, 1.083, 27.334),
                        new PoseScaled(8.484, 0.568, 3.911)), 
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                    RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path B red");
                        
                    Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                    return new SequentialCommandGroup(
                        new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                        new LowerArm(),
                        new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                            
                }
                else
                {
                    //blue path
                    //new PoseScaled(4.088, -1.503, -1.011)
                    //new PoseScaled(4.500, 0.464, 88.462)
                    //new PoseScaled(5.712, 0.261, -48.560)
                    //new PoseScaled(7.600, -2.011, -44.253)
                    //new PoseScaled(9.190, -2.380, -1.362)
                    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                        new PoseScaled(0, 0, 0),
                        new PoseScaled(3.898, 0.028, -1.099), //D6
                        new PoseScaled(5.604, 1.312, 6.987), //B8
                        new PoseScaled(7.094, 0.237, -39.243)), //D10
                        VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path B blue");
    
                    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                        endPose(trajectory1), // 
                        new PoseScaled(8.594, -0.084, -10.723)),
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                    RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path B blue");
                        
                    /*new PoseScaled(0, 0, 0),
                        new PoseScaled(3.993, -1.643, -23.467), //D6
                        new PoseScaled(5.706, -0.314, 46.274), //B8
                        new PoseScaled(7.281, -1.515, -23.203)), //D10
                        endPose(trajectory1), //D10
                        new PoseScaled(9.073, -2.231, -7.031)), //new PoseScaled(8.815, -0.222, 11.338)
                    */

                    
                    Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                    return new SequentialCommandGroup(
                        new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                        new LowerArm(),
                        new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                            
                }

   
   
            }
            
                default:
                    return null;
        }
    }

    public static Command getAutoCommandIRAHCompetitionBotGalacticPigeon(int selectorOne)
    {
        System.out.println("getAutoCommandIRAHCompetitionBotGalacticPigeon SelectorOne: "+selectorOne);
        switch (selectorOne) {
            case 0:
                return null;

            //Using the gyroheading angle to decide the red or blue path
            //For the red path, the robot faces towards the first power cell so the angle is zero
            //For the blue path, the robot faces towards the first power cell so the angle is around -20
            //For both paths, the robot starts from the same location

            case 1:
                {
                    //Galactic: path A
                    //first determine if the red or blue path
                    double currentAngle = DriveBaseHolder.getInstance().getPose().getRotation().getDegrees();
                    System.out.println("currentAngle path A:"+currentAngle);
                    boolean bRed = false;
                    if ( currentAngle > -10 )
                    {
                       bRed = true;
                    }

                    //Hardcoded, will be removed later
                    System.out.println("Path A, bRed: " + bRed);
   
                    if ( bRed )
                    {
                        //red path
                        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                            new PoseScaled(0, 0, 0),
                            new PoseScaled(1.062, 0.001, 0.220),    //C3 
                            new PoseScaled(3.251, -0.731, -28.521), //D5  (y=-0.631)
                            new PoseScaled(4.127, 1.060, 83.013)),  //A6
                            VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                        RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path A red heading");
    
                        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                            endPose(trajectory1), //A6
                            new PoseScaled(5.331, 1.608, 1.978),
                            new PoseScaled(8.479, 1.769, 4.702)),
                            VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                        RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path A red heading");
                        
                        
                        Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                        return new SequentialCommandGroup(
                            new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                            new LowerArm(),
                            new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                        
                    }
                    else
                    {
                        //blue path
                        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                            new PoseScaled(-0.039, 0.010, -17.095),//
                            new PoseScaled(4.201, -0.985, -0.264), //E6 
                            new PoseScaled(5.170, 0.529, 73.389), //B7 
                            new PoseScaled(6.597, 0.627, -87.759)), //C9
                            VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                        RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path A blue heading");
    
                        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                            endPose(trajectory1), //C9
                            new PoseScaled(8.747, -0.777, 2.285)),
                            VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                        RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path A blue heading");
                        
                        
                        Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                        return new SequentialCommandGroup(
                            //new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters),
                            new LowerArm(),
                            new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                        
                    }
                }
            case 2:
            {
                //Galactic: path B
                //first determine if the red or blue path
                double currentAngle = DriveBaseHolder.getInstance().getPose().getRotation().getDegrees();
                System.out.println("currentAngle path A:"+currentAngle);
                boolean bRed = false;
                if ( currentAngle > -10 )
                {
                   bRed = true;
                }

                System.out.println("Path B, bRed: "+bRed);
   
                if ( bRed )
                {
                    //red path
                    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                        new PoseScaled(0, 0, 0),
                        new PoseScaled(1.614, 0.060, 1.978),   //B3
                        new PoseScaled(3.123, -1.2, -42.539),  //D5
                        new PoseScaled(4.142, -2.309, 6.680),  
                        new PoseScaled(5.3, -0.288, 53.394)),  //B7
                        VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path B red heading");
    
                    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                        endPose(trajectory1), //B7 
                        new PoseScaled(6.034, 1.083, 27.334),
                        new PoseScaled(8.484, 0.568, 3.911)), 
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                    RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path B red heading");
                        
                    
                    Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                    return new SequentialCommandGroup(
                        new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                        new LowerArm(),
                        new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                            
                }
                else
                {
                    //blue path
                    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                        new PoseScaled(0.005, -0.000, -20.566),
                        new PoseScaled(3.796, -1.502, -0.220), //D6
                        new PoseScaled(5.331, -0.6, 4.395), //B8 -0.241
                        new PoseScaled(6.888, -1.224, -41.201)), //D10
                        VisionPose.getInstance().getTrajConfig(0, Config.kRamseteGalacticSpeed, false));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "Galactic Path B blue heading");
    
                    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                        endPose(trajectory1), // 
                        new PoseScaled(9.054, -1.622, 0.791)),
                        VisionPose.getInstance().getTrajConfig(Config.kRamseteGalacticSpeed, 0, false));
                    RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "Galactic Path B blue heading");
                    
                    Command rameseteCommands = new SequentialCommandGroup(ramsete1, ramsete2);
                    return new SequentialCommandGroup(
                        new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                        new LowerArm(),
                        new ParallelRaceGroup(new AutoIntakeCommand(), rameseteCommands));
                            
                }
   
            }
            

                case 3: {
                    // BarrelRacing
                    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                        new PoseScaled(0.000, 0.000, 0.0),
                        new PoseScaled(2.954, 0.016, -0.791),
                        new PoseScaled(4.008, -0.925, -90.615),
                        new PoseScaled(2.927, -1.713, 176.660),
                        new PoseScaled(1.935, -0.884, 90.220),
                        new PoseScaled(2.973, 0.059, -2.109),
                        new PoseScaled(5.311, -0.237, 1.187),
                        new PoseScaled(6.345, 0.588, 92.988),
                        new PoseScaled(5.300, 1.481, -174.067),
                        new PoseScaled(4.391, 0.510, -80.859),
                        new PoseScaled(5.460, -1.291, -27.378),
                        new PoseScaled(6.848, -1.751, 3.999),
                        new PoseScaled(7.835, -0.916, 96.987),
                        new PoseScaled(6.751, -0.009, -171.958),
                        new PoseScaled(5.992, -0.356, -149.326),
                        new PoseScaled(2.940, -0.365, -172.002),
                        new PoseScaled(-0.099, -0.35, -180)),
                        VisionPose.getInstance().getTrajConfig(0, 0, false));
                    RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAHComp-BarrelRacing");

                    return new SequentialCommandGroup(
                            new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.sample(0).poseMeters)),
                            new LowerArm(),
                            ramsete1);
                }

                case 4: {
                    // Slolam
                    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                    new PoseScaled(0.300, 0.000, -180),
                    new PoseScaled(0.902, 0.165, -138.955),
                    new PoseScaled(1.301, 0.947, -98.921),
                    new PoseScaled(1.667, 1.8, -139.087),
                    new PoseScaled(3.294, 2.0, 179.297),
                    new PoseScaled(5.271, 2.0, 159.478),
                    new PoseScaled(5.974, 1.289, 88.110),
                    new PoseScaled(6.652, 0.234, -179.561),
                    new PoseScaled(7.597, 1.148, -84.595),
                    new PoseScaled(6.8, 2.078, 4.131),
                    new PoseScaled(6.4, 1.169, 94.175),
                    new PoseScaled(5.4, 0.144, 10),
                    new PoseScaled(2.4, 0.129, 0.835),
                    new PoseScaled(2.2, 0.75, -89.868),
                    new PoseScaled(0.448, 0.9, 4.043),
                    new PoseScaled(-0.824, 1.0, 3.604)),
                    VisionPose.getInstance().getTrajConfig(0, 0, true));
            RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "IRAHPrac-Slolam-P1");
            return new SequentialCommandGroup (
                new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(trajectory1.getInitialPose())),
                new LowerArm(),
                ramsete1
            );//.alongWith(new LowerArm());
        }
                

                default:
                    return null;
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
