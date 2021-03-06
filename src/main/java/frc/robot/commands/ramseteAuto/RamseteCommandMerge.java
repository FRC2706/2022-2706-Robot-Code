/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Spreadsheet record of the index of each logging column:
// https://docs.google.com/spreadsheets/d/1wX1cwLKulJhgrL0wivAJ1hezfs7Bp6M6NHVlwM9hJ3g/edit?usp=sharing

package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.subsystems.DriveBase.DriveBaseState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.commands.ramseteAuto.SimpleCsvLogger;

/**
 * This command was copied and modified from RamseteCommand on WpiLib.
 * https://github.com/wpilibsuite/allwpilib/blob/master/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/RamseteCommand.java
 * 
 * A command that uses a RAMSETE controller ({@link RamseteController}) to
 * follow a trajectory {@link Trajectory} with a differential drive.
 *
 * <p>
 * The command handles trajectory-following, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by
 * teams without a great deal of controls expertise.
 *
 * <p>
 * Advanced teams seeking more flexibility (for example, those who wish to use
 * the onboard PID functionality of a "smart" motor controller) may use the
 * secondary constructor that omits the PID returning only the raw wheel speeds
 * from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandMerge extends CommandBase {
    private final Timer m_timer = new Timer();
    private Trajectory m_trajectory;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_totalTimeElapsed = 0;
    private double m_timeBeforeTrajectory = 0;
    private double m_prevTime;
    private Pose2d targetPose;
    private final DriveBase m_driveSubsystem;

    // NetworkTable Values
    private NetworkTableEntry xError, yError, rotError, xCurrent, yCurrent, rotCurrent;

    // USB Logger
    private SimpleCsvLogger usbLogger;
    private String loggingDataIdentifier;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided
     * trajectory. Performs P control and calculates feedforwards; outputs are the
     * raw wheel speeds from the RAMSETE controller and the feedforwards. It will
     * follow the full trajectory
     *
     * @param trajectory The trajectory to follow.
     */
    public RamseteCommandMerge(Trajectory trajectory, String loggingDataIdentifier) {
       
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommandMerge");

        m_driveSubsystem = DriveBaseHolder.getInstance();
        m_follower = new RamseteController(Config.kRamseteB, Config.kRamseteZeta);
        m_kinematics = Config.kDriveKinematics;

        m_feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter,
                Config.kaVoltSecondsSquaredPerMeter);

        if(m_driveSubsystem != null)
        {
            addRequirements(m_driveSubsystem);
        }

        usbLogger = new SimpleCsvLogger();
        this.loggingDataIdentifier = loggingDataIdentifier;

        var table = NetworkTableInstance.getDefault().getTable("RamseteAuto");
        xError = table.getEntry("xError");
        yError = table.getEntry("yError");
        rotError = table.getEntry("rotError"); 

        xCurrent = table.getEntry("xCurrent");
        yCurrent = table.getEntry("yCurrnet");
        rotCurrent = table.getEntry("rotCurrent"); 

    }

    @Override
    public void initialize() {

        if(m_driveSubsystem.getDriveBaseState() != DriveBaseState.Degraded)
        {
            m_prevTime = 0;
            var initialState = m_trajectory.sample(0);
            m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
            m_timer.reset();
            m_timer.start();

            startLogging();
            targetPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;

            m_driveSubsystem.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_RAMSETE);
            m_driveSubsystem.setCoastMode();
        }
    }

    @Override
    public void execute() {

        if(m_driveSubsystem.getDriveBaseState() == DriveBaseState.Degraded)
        {
            System.out.println("RamseteCommandMerge: DriveBase is in degraded mode: Do nothing");
        }
        else
        {
            m_totalTimeElapsed = m_timer.get();
            double curTime = m_totalTimeElapsed - m_timeBeforeTrajectory;
            double dt = m_totalTimeElapsed - m_prevTime;

            Pose2d currentPose = m_driveSubsystem.getPose();
            Trajectory.State desiredState = m_trajectory.sample(curTime);

            // Exclusively used for logging
            Pose2d poseError = desiredState.poseMeters.relativeTo(currentPose);
            xError.setNumber(poseError.getTranslation().getX());
            yError.setNumber(poseError.getTranslation().getY());
            rotError.setNumber(poseError.getRotation().getDegrees());

            xCurrent.setNumber(currentPose.getTranslation().getX());
            yCurrent.setNumber(currentPose.getTranslation().getY());
            rotCurrent.setNumber(currentPose.getRotation().getDegrees());

            var targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(currentPose, desiredState));

            double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
            double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

            double leftAcceleration = (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt;
            double rightAcceleration = (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt;

            double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint, leftAcceleration);
            double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint, rightAcceleration);

            m_driveSubsystem.tankDriveVelocities(leftSpeedSetpoint, rightSpeedSetpoint,
                    leftFeedforward, rightFeedforward);

            m_prevTime = m_totalTimeElapsed;
            m_prevSpeeds = targetWheelSpeeds;

            // Get measure velocities for logging
            double measuredVelocities[] = m_driveSubsystem.getMeasuredVelocities();

            // Log Data - See Spreadsheet link at top
            logData(m_totalTimeElapsed, desiredState.poseMeters.getTranslation().getX(),
                    desiredState.poseMeters.getTranslation().getY(), desiredState.poseMeters.getRotation().getDegrees(),
                    desiredState.velocityMetersPerSecond, desiredState.accelerationMetersPerSecondSq,
                    desiredState.curvatureRadPerMeter, currentPose.getTranslation().getX(),
                    currentPose.getTranslation().getY(), currentPose.getRotation().getDegrees(),
                    poseError.getTranslation().getX(), poseError.getTranslation().getY(),
                    poseError.getRotation().getDegrees(), leftSpeedSetpoint, rightSpeedSetpoint, measuredVelocities[0],
                    measuredVelocities[1], leftFeedforward, rightFeedforward, 
                    leftAcceleration, rightAcceleration, targetPose.getTranslation().getX(),
                    targetPose.getTranslation().getY(), targetPose.getRotation().getDegrees());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // if (isFinished() == false) {
        //     m_driveSubsystem.setBrakeMode();
        // }
        m_timer.stop();
        stopLogging();
        m_driveSubsystem.setActivePIDSlot(Config.DRIVETRAIN_SLOTID_DRIVER);
    }

    @Override
    public boolean isFinished() {
        if(m_driveSubsystem.getDriveBaseState() == DriveBaseState.Degraded)
            return true;
        else
            return m_timer.advanceIfElapsed(m_trajectory.getTotalTimeSeconds() + m_timeBeforeTrajectory);
      
     }

    public double getTotalTime() {
        return m_trajectory.getTotalTimeSeconds();
    }

    public double getElapsedTime() {
        return m_totalTimeElapsed;
    }

    public void setNewTrajectory(Trajectory newTrajectory) {
        m_trajectory = newTrajectory;
        m_timeBeforeTrajectory = m_timer.get();
        targetPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;
    }

    public Pose2d getTargetPose() {
        return m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;
    }

    /**
     * All Methods used for USB logging startLogging() logData(data) stopLogging()
     */
    public void startLogging() {
        // See Spreadsheet link at top
        usbLogger.init(loggingDataIdentifier, new String[] { "TrajectoryTime", "stateX", "stateY", "stateRot", //
                "stateVel", "stateAccel", "stateCurv", //
                "currentX", "currentY", "currentRot", //
                "errorX", "errorY", "errorRot", //
                "cmdLeftVel", "cmdRightVel", "measLeftVel", "measRightVel", //
                "leftFF", "rightFF", //
                "leftSetpointAccel", "rightSetpointAccel", //
                "targetX", "targetY", "targetRot" },
                new String[] { "s", "m", "m", "deg", //
                        "m/s", "m/s/s", "rad/s", //
                        "m", "m", "deg", //
                        "m", "m", "deg", //
                        "m/s", "m/s", "m/s", "m/s", //
                        "%", "%",  //
                        "m/s/s", "m/s/s", //
                        "m", "m", "deg" });
    }

    public void logData(double... data) {
        usbLogger.writeData(data);
    }

    public void stopLogging() {
        usbLogger.close();
    }

}