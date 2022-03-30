/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DetectCargoAtIndexer;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.config.Config;
import frc.robot.nettables.ControlCtrlNetTable;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBase2020;
import frc.robot.subsystems.DriveBaseHolder;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import java.io.IOException;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.logging.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    static Logger logger = Logger.getLogger("Robot");
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    //network table for vision control
    private VisionCtrlNetTable visionControlNetTable;
    //network table for control systems control
    private ControlCtrlNetTable controlCtrlNetTable;
    //flag to indicate from the teleop mode
    private Boolean bFromTeleMode;
    //flag to indicate the real match
    private Boolean bRealMatch;

    public static boolean m_bAutoMode = false;

    //@todo: clean up
    //imported trajectory
    //on RoboRio at "/home/lvuser/deploy";
    String trajectoryJSON = "./paths/scaled3.wpilib.json";
    public static Trajectory trajectoryRead = new Trajectory();

    String blue3Option1p1 = "./paths/Blue3Option1P1.json";
    public static Trajectory trajectoryBlue3O1P1 = new Trajectory();

    String blue3Option1p2 = "./paths/Blue3Option1P2.json";
    public static Trajectory trajectoryBlue3O1P2 = new Trajectory();

    String blueOption2 = "./paths/BlueOption2.json";
    public static Trajectory trajectoryBlueO2 = new Trajectory();

    String blueOption3 = "./paths/BlueOption3.json";
    public static Trajectory trajectoryBlueO3 = new Trajectory();

    String redOption1 = "./paths/RedOption1.json";
    public static Trajectory trajectoryRedO1 = new Trajectory();

    String redOption2 = "./paths/RedOption2.json";
    public static Trajectory trajectoryRedO2 = new Trajectory();

    String redOption3 = "./paths/RedOption3.json";
    public static Trajectory trajectoryRedO3 = new Trajectory();

    String blueOption4P1 = "./paths/4OptionP1Blue.wpilib.json";
    public static Trajectory trajectoryBlue4OP1 = new Trajectory();

    String blueOption4P2 = "./paths/4OptionP2Blue.wpilib.json";
    public static Trajectory trajectoryBlue4OP2 = new Trajectory();

    String blueOption4P3 = "./paths/4OptionP3Blue.wpilib.json";
    public static Trajectory trajectoryBlue4OP3 = new Trajectory();

    String blueOption8P1 = "./paths/8BluePath1.wpilib.json";
    public static Trajectory trajectoryBlue8P1 = new Trajectory();

    String blueOption8P2 = "./paths/8BluePath2.wpilib.json";
    public static Trajectory trajectoryBlue8P2 = new Trajectory();


    /**
     * Determines if the robot is in a real match.
     *
     * @return True if the robot is in a real match, false otherwise.
     */
    public static boolean isRealMatch() {
        return DriverStation.isFMSAttached();
    }

    /**
     * Halts the robot in every way, add a demo comment
     */
    public static void haltRobot(String s) {
        /*
         Disable the scheduler and cancel everything.
         This stops all subsystems and commands and prevents new ones from starting
        */
        var scheduler = CommandScheduler.getInstance();
        scheduler.disable();
        scheduler.cancelAll();

        /*
         Log the message and close the log file.
         Not strictly necessary, but it's good practice to close files when you're done with it.
        */
        logger.severe("ROBOT HALTED: " + s);
        Config.logFileHandler.close();

        /*
         Hang (Using Thread.sleep instead of a busy loop).
         The reason we want to hang instead of letting the robot code exit is purely because the RoboRIO is going to
         start the code up again, so to avoid creating many dozens of log files in the 2 minutes it takes to fix the
         robot, we'll just hang and let the operator restart the code when they're ready.
        */
        try {
            while (true) {
                Thread.sleep(1000);
            }
        } catch (Exception e) {
            // If the sleep gets interrupted we'll let the robot code restart.
        }
    }

    /**
     * Same as above but takes an exception for printing
     *
     * @param s A String to print
     * @param e the exception to print
     */
    public static void haltRobot(String s, Exception e) {
        // Format the exception as a string
        StringWriter sw = new StringWriter();
        e.printStackTrace(new PrintWriter(sw));
        String stackTrace = sw.toString();

        // Call the string only version
        haltRobot(s + "\n" + stackTrace);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        IndexerSubSystem.getInstance().setDefaultCommand(new DetectCargoAtIndexer());
        
        //read a trajectory 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectoryRead = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            
            Path PathB3O1P1 = Filesystem.getDeployDirectory().toPath().resolve(blue3Option1p1);
            trajectoryBlue3O1P1 = TrajectoryUtil.fromPathweaverJson(PathB3O1P1);

            Path PathB3O1P2 = Filesystem.getDeployDirectory().toPath().resolve(blue3Option1p2);
            trajectoryBlue3O1P2 = TrajectoryUtil.fromPathweaverJson(PathB3O1P2);

            Path PathB3O2 = Filesystem.getDeployDirectory().toPath().resolve(blueOption2);
            trajectoryBlueO2 = TrajectoryUtil.fromPathweaverJson(PathB3O2);

            Path PathB3O3 = Filesystem.getDeployDirectory().toPath().resolve(blueOption3);
            trajectoryBlueO3 = TrajectoryUtil.fromPathweaverJson(PathB3O3);

            Path PathR3O1 = Filesystem.getDeployDirectory().toPath().resolve(redOption1);
            trajectoryRedO1 = TrajectoryUtil.fromPathweaverJson(PathR3O1);

            Path PathR3O2 = Filesystem.getDeployDirectory().toPath().resolve(redOption2);
            trajectoryRedO2 = TrajectoryUtil.fromPathweaverJson(PathR3O2);

            Path PathR3O3 = Filesystem.getDeployDirectory().toPath().resolve(redOption3);
            trajectoryRedO3 = TrajectoryUtil.fromPathweaverJson(PathR3O3);

            Path PathB4O1P1 = Filesystem.getDeployDirectory().toPath().resolve(blueOption4P1);
            trajectoryBlue4OP1 = TrajectoryUtil.fromPathweaverJson(PathB4O1P1);

            Path PathB4O1P2 = Filesystem.getDeployDirectory().toPath().resolve(blueOption4P2);
            trajectoryBlue4OP2 = TrajectoryUtil.fromPathweaverJson(PathB4O1P2);

            Path PathB4O1P3 = Filesystem.getDeployDirectory().toPath().resolve(blueOption4P3);
            trajectoryBlue4OP3 = TrajectoryUtil.fromPathweaverJson(PathB4O1P3);

            Path PathB8P1 = Filesystem.getDeployDirectory().toPath().resolve(blueOption8P1);
            trajectoryBlue8P1 = TrajectoryUtil.fromPathweaverJson(PathB8P1);

            Path PathB8P2 = Filesystem.getDeployDirectory().toPath().resolve(blueOption8P2);
            trajectoryBlue8P2 = TrajectoryUtil.fromPathweaverJson(PathB8P2);
          

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        DriveBaseHolder.init();
        m_robotContainer = new RobotContainer();
        //create a vision control table
        visionControlNetTable = new VisionCtrlNetTable();
        //create a control system control table
        controlCtrlNetTable = new ControlCtrlNetTable();
        //set to false
        bFromTeleMode = false;
        bRealMatch = false;

        logger.addHandler(Config.logFileHandler);

        DriverStation.silenceJoystickConnectionWarning(true);
        
        VisionPose.getInstance().initVision(VisionPose.VisionType.TPracticeTarget);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        // SmartDashboard.putNumber("Outer Port Yaw", VisionCtrlNetTable.yawToOuterPort.get());
        // SmartDashboard.putNumber("PowerCell Distance", VisionCtrlNetTable.distanceToPowerCell.get());
        // SmartDashboard.putNumber("Pigeon Yaw", DriveBaseHolder.getInstance().getOdometryHeading().getDegrees());

        m_bAutoMode = DriverStation.isAutonomous();
        
        CommandScheduler.getInstance().run();

        //The following 2 lines run Drivebase methods that tell shuffleboard what the motor current draw is and if motor current limiting is active.
        DriveBaseHolder.getInstance().getMotorCurrent();
        DriveBaseHolder.getInstance().isMotorLimitActive();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        if (bRealMatch && bFromTeleMode) {
            // if in a real match and from teleop mode
            // Write to the network table the shut down signal.
            visionControlNetTable.shutDownVision();
            controlCtrlNetTable.shutDownControl();

            System.out.println("Driver Station Disabled");
        }

    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        //send start up signal to vision team
        visionControlNetTable.startUpVision();

        bFromTeleMode = false;
        bRealMatch = isRealMatch();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        ArmSubsystem.getInstance().stopMotor();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // VisionPose.getInstance().test();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        //set teleop mode to false
        bFromTeleMode = false;

        VisionPose.getInstance().test();


    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        //set teleop mode to true
        bFromTeleMode = true;
    }
}
