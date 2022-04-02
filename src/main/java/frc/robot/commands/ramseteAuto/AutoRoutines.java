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
import frc.robot.commands.IndexerCargo;
import frc.robot.commands.IndexerForShooter;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.KickerFloat;
import frc.robot.commands.OuterGoalErrorLoop;
import frc.robot.commands.RunIntakeCargo;
import frc.robot.commands.ControlKicker;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBaseHolder;
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
                //steps: low goal -> taxi
                Command wait1s1 = new WaitCommand(1);
                Command delayIndexer1 = wait1s1.andThen( new IndexerForShooter());
                Command autoShootlow1 = new ParallelRaceGroup(new SpinUpShooterWithTime(1800, 4), delayIndexer1);
                Command kicker1 = new ControlKicker(true);
                Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0.0,0.0,new Rotation2d()), new Pose2d(1.0,0.0, new Rotation2d())), Config.trajectoryConfig);
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(traj1, "Trajectory-Red-O1");
                return new SequentialCommandGroup ( new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(traj1.getInitialPose())),
                                                    new ParallelRaceGroup(kicker1, new WaitCommand(0.5)),                                   
                                                    autoShootlow1,
                                                    new ParallelRaceGroup(ramsete1));
            
            case 2:
                //steps: low goal -> taxi -> intake one cargo -> high goal -> stow intake
                Command wait1s2 = new WaitCommand(1);
                Command delayIndexer2 = wait1s2.andThen( new IndexerForShooter());
                Command autoShootlow2 = new ParallelRaceGroup(new SpinUpShooterWithTime(1800, 4), delayIndexer2);
                Command wait1s22 = new WaitCommand(1);
                Command delayIndexer22 = wait1s22.andThen( new IndexerForShooter());
                Command autoShoothigh2 = new ParallelRaceGroup(new SpinUpShooterWithTime(3100, 8), delayIndexer22);
                Command intakeDown2 = new IntakeDown();
                Command intakeUp2 = new IntakeUp();
                Command kicker3 = new ControlKicker(true);
                Trajectory traj2 = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0.0,0.0,new Rotation2d()), new Pose2d(1.3,0.0, Rotation2d.fromDegrees(-4))), Config.trajectoryConfig);
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(traj2, "Trajectory-Red-O2");
                
                return new SequentialCommandGroup ( new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(traj2.getInitialPose())),
                                                    new ParallelRaceGroup(kicker3, new WaitCommand(0.5)),
                                                    autoShootlow2,
                                                    new ParallelRaceGroup(intakeDown2, new WaitCommand(0.5)),
                                                    new ParallelRaceGroup(ramsete2, new RunIntakeCargo(true, 4)),
                                                    new InstantCommand(DriveBaseHolder.getInstance()::setBrakeMode),
                                                    autoShoothigh2.alongWith(new RunIntakeCargo(true, 4)),
                                                    intakeUp2);

            case 3:
                //steps: kicker up -> high goal -> taxi -> kicker down -> 
                //       intake one cargo -> high goal -> stow intake
                Command wait1s3 = new WaitCommand(1);
                Command delayIndexer3 = wait1s3.andThen( new IndexerForShooter());
                Command autoShoot3 = new ParallelRaceGroup(new SpinUpShooterWithTime(2750, 4), delayIndexer3);
                Command wait1s4 = new WaitCommand(1);
                Command delayIndexer4 = wait1s4.andThen( new IndexerForShooter());
                Command autoShoot4 = new ParallelRaceGroup(new SpinUpShooterWithTime(3100, 8), delayIndexer4);
                Command intakeDown = new IntakeDown();
                Command intakeUp = new IntakeUp();
                Command kicker = new ControlKicker(false);
                Command kicker2 = new ControlKicker(true);
                Command kickerfloat = new KickerFloat();

                Trajectory traj = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0.0,0.0,new Rotation2d()), new Pose2d(0.65,0.0, new Rotation2d())), Config.trajectoryConfig);
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(traj, "Trajectory-Red-O2");
                return new SequentialCommandGroup ( new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(traj.getInitialPose())),
                                                  new ParallelRaceGroup(kicker, new WaitCommand(0.5)),
                                                  autoShoot3,
                                                  new ParallelRaceGroup(intakeDown, new WaitCommand(0.5)),
                                                  new ParallelRaceGroup(ramsete3, new RunIntakeCargo(true, 4)),
                                                  new InstantCommand(DriveBaseHolder.getInstance()::setBrakeMode),
                                                  new ParallelRaceGroup(kicker2, new WaitCommand(0.5)),
                                                  autoShoot4.alongWith(new RunIntakeCargo(true, 4)),
                                                  kickerfloat,
                                                  intakeUp);
           
        case 4:
                //description:
                //starting position: within tarmac and facing a red cargo
                // if using odometry: middle red cargo
                //drive forward first --> pick up 2nd cargo --> shoot both cargo
                // RamseteCommandMerge ramsete7 = new RamseteCommandMerge(Robot.trajectoryRedO2, "Trajectory-Red-O2");
                // //Command pickUpCargo = new RunIntakeCargo(true)

                // return new SequentialCommandGroup (
                //     new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryRedO2.getInitialPose())),
                //     ramsete7);

                // RamseteCommandMerge ramsete4 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Blue3-O1P1");
                //     return new SequentialCommandGroup (
                //         new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                //         ramsete4);

                //steps: low goal -> taxi -> intake one cargo -> high goal -> stow intake
                Command wait1s7 = new WaitCommand(1.5);
                Command delayIndexer7 = wait1s7.andThen( new IndexerForShooter());
                Command autoShoothigh3 = new ParallelRaceGroup(new SpinUpShooterWithTime(3150, 8), delayIndexer7);
                Command intakeDown3 = new IntakeDown();
                Command intakeUp3 = new IntakeUp();
                Command kicker4 = new ControlKicker(true);
                Trajectory traj3 = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0.0,0.0,new Rotation2d()), new Pose2d(1.3,0.0, Rotation2d.fromDegrees(-4))), Config.trajectoryConfig);
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(traj3, "Trajectory-Red-O2");
                
                return new SequentialCommandGroup ( new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(traj3.getInitialPose())),
                                                    new ParallelRaceGroup(kicker4, new WaitCommand(0.5)),
                                                    new ParallelRaceGroup(intakeDown3, new WaitCommand(0.5)),
                                                    new ParallelRaceGroup(ramsete4, new RunIntakeCargo(true, 4)),
                                                    new InstantCommand(DriveBaseHolder.getInstance()::setBrakeMode),
                                                    autoShoothigh3.alongWith(new RunIntakeCargo(true, 4)),
                                                    intakeUp3);

            case 5:
                //description:
                //starting position: within tarmac and facing a red blue
                // if using odometry: middle blue cargo
                //drive forward first --> pick up 2nd cargo --> shoot both cargo
                RamseteCommandMerge ramsete5 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Blue-O1-P1");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete5);

    
            case 6:
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
                //testing red option 2
                RamseteCommandMerge ramsete1 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Red-O2");
                return new SequentialCommandGroup (
                    //Shoot first 
                    //new SpinUpShooterWithTime(2000, 0)
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete1);
                    //new ParallelCommandGroup(ramsete1, new RunIntakeCargo(2),
                   // new SpinUpShooterWithTime(2500, 0));
                    
            case 2:
                //testing blue option 2
                RamseteCommandMerge ramsete2 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Blue-O2");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete2);
            case 3:
                RamseteCommandMerge ramsete3 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Red-O2");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete3);
            case 4:
                RamseteCommandMerge ramsete4 = new RamseteCommandMerge(Robot.trajectoryBlue3O1P1, "Trajectory-Blue-O2");
                return new SequentialCommandGroup (
                    new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose(Robot.trajectoryBlue3O1P1.getInitialPose())),
                    ramsete4);
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
                   // new InstantCommand(() -> FeederSubsystem.getInstance().setBallsAroundFeeder(0)),
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
