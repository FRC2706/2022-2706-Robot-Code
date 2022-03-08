/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.config.Config;
import frc.robot.sensors.AnalogSelector;
import frc.robot.subsystems.*;
import frc.robot.commands.ramseteAuto.AutoRoutines;
import frc.robot.commands.ramseteAuto.DriveToWaypoint;
import frc.robot.commands.ramseteAuto.PassThroughWaypoint;
import frc.robot.commands.ramseteAuto.PoseScaled;
import frc.robot.commands.ramseteAuto.RamseteCommandMerge;
import frc.robot.commands.ramseteAuto.TranslationScaled;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
//import frc.robot.commands.ramseteAuto.ReadPath;

import frc.robot.nettables.VisionCtrlNetTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.List;
import java.util.logging.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    
    // RobotContainer is a singleton class
    private static RobotContainer currentInstance;

  // The robot's subsystems and commands are defined here...    
  private Joystick driverStick;
  private Joystick controlStick;
  public AnalogSelector analogSelectorOne;
  private AnalogSelector analogSelectorTwo;
  private Command driveCommand;
  private Command intakeCommand;
  private Command reverseFeeder;
  private Command moveToOuterPort;
  private Command reverseArmManually;
  private Command positionPowercell;
  private Command rampShooterCommand;
  private Command incrementFeeder;
  private Command moveArm;
  private Command sensitiveDriving;
  private Logger logger = Logger.getLogger("RobotContainer");
  private final double AUTO_DRIVE_TIME = 1.0;
  private final double AUTO_LEFT_MOTOR_SPEED = 0.2;
  private final double AUTO_RIGHT_MOTOR_SPEED = 0.2;
  private Command runFeeder;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
       // UltrasoundSensor sensor = new UltrasoundSensor();

        // Configure the button bindings
        logger.addHandler(Config.logFileHandler);
        if (Config.ANALOG_SELECTOR_ONE != -1) {
            analogSelectorOne = new AnalogSelector(Config.ANALOG_SELECTOR_ONE);
        }

        ArmSubsystem armSubsystem;
        if (Config.ARM_TALON != -1)
            armSubsystem = ArmSubsystem.getInstance();

        configureButtonBindings();

        // Only construct the RelaySubsystem if it has relays which is only on mini bot
        // Atm the only way to tell if its the mini bot is if it has follower motors
        if (Config.robotId == 2) {
            RelaySubsystem.getInstance();
        }

    
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverStick = new Joystick(0);
        controlStick = new Joystick(1);
      
        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS, false);
        DriveBaseHolder.getInstance().setDefaultCommand(driveCommand);

        sensitiveDriving = new SensitiveDriverControl(driverStick);
        new JoystickButton(driverStick, XboxController.Button.kLeftBumper.value).whenHeld(sensitiveDriving);
 
        // Command resetHeading = new InstantCommand(() -> DriveBaseHolder.getInstance().resetHeading(Rotation2d.fromDegrees(0)));
        // new JoystickButton(driverStick, XboxController.Button.kStart.value).whenActive(resetHeading);

        //@todo: put the robot at the same place whenever we start a new path
        Command resetPose = new InstantCommand(() -> DriveBaseHolder.getInstance().resetPose( new Pose2d()));
        new JoystickButton(driverStick, XboxController.Button.kStart.value).whenActive(resetPose);
        
        switch ( Config.robotId )
        {
            case 0:
            {
                // Instantiate the intake command and bind it
                intakeCommand = new OperatorIntakeCommand();
                new JoystickButton(controlStick, XboxController.Button.kLeftBumper.value).whenHeld(intakeCommand);
              
                positionPowercell = new PositionPowercellCommand();
                new JoystickButton(controlStick, XboxController.Button.kRightBumper.value).toggleWhenActive(positionPowercell, true);
                
                reverseFeeder = new ReverseFeeder();
                new JoystickButton(controlStick, XboxController.Button.kB.value).whenHeld(reverseFeeder);
                
                incrementFeeder = new IncrementFeeder(-FeederSubsystem.FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
                new JoystickButton(controlStick, XboxController.Button.kX.value).whenHeld(incrementFeeder);
                
                moveToOuterPort = new TurnToOuterPortCommand(true, 3.0, 2.0);
                new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(moveToOuterPort, true);
                
                if (Config.ARM_TALON != -1) {
                    reverseArmManually = new MoveArmManuallyCommand(-0.35);
                    new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(reverseArmManually);
        
                    moveArm = new MoveArmManuallyCommand(10);
                    new JoystickButton(driverStick, XboxController.Button.kY.value).whenHeld(moveArm);
        
                    Command lowerArm = new LowerArm();
                    // new JoystickButton(driverStick, XboxController.Button.kB.value).whenActive(lowerArm);
                }
        
                if (Config.FEEDER_SUBSYSTEM_TALON != -1) {
                    // Set default command of feeder to index when limit is pressed
                    Command indexFeeder = new IndexBall().andThen(new DoNothingForSeconds(1.5));
                    Command pollInputSwitch = new PollLimitSwitch(indexFeeder, FeederSubsystem.getInstance(), FeederSubsystem::isBallAtInput);
                    FeederSubsystem.getInstance().setDefaultCommand(pollInputSwitch); 
                }
                break;
            }
            case 1: 
            {
                break;
            }
            case 2: //Beetle
            {
                //Front ring light
                Command controlFrontRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_FRONT);
                new JoystickButton(driverStick, XboxController.Button.kRightBumper.value).whenPressed(controlFrontRinglight);
                
                //Rear small ring light
               // Command controlRearSmallRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_REAR_SMALL);
               // new JoystickButton(driverStick, XboxController.Button.kX.value).whenPressed(controlRearSmallRinglight);
                
                //Command printX = new PrintOdometry();
                //new JoystickButton(driverStick, XboxController.Button.kX.value).whenPressed(printX);

                //Rear large ring light
               // Command controlRearLargeRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_REAR_LARGE);
                //new JoystickButton(driverStick, XboxController.Button.kY.value).whenPressed(controlRearLargeRinglight);
                        
                //Read a trajectory
                // Command readTrajectory = new ReadPath( Robot.trajectoryRead, "Slalom path");
                // new JoystickButton(driverStick, XboxController.Button.kB.value).whenPressed(readTrajectory);

                // Command readIrSensor = new ReadAnalogInput(7);
                // new JoystickButton(driverStick, XboxController.Button.kA.value).whenPressed(readIrSensor);

               // Command readSwitch = new TestSwitch();
                //new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(readSwitch);

            //    Command readColorSensor = new ReadColorSensor();
             //   new JoystickButton(driverStick, XboxController.Button.kB.value).whenPressed(readColorSensor);

                //for shooter command
                Command wait1s = new WaitCommand(1);
                Command delayIndexer = wait1s.andThen( new IndexerForShooter());
                Command shooter = new ParallelCommandGroup(new SpinUpShooterWithTime(2000, 10), delayIndexer);
                new JoystickButton(driverStick, XboxController.Button.kB.value).whenHeld(shooter);

                //Command testIndexer = new TestIndexer();
                //new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(testIndexer);

                //indexer cargo command
                Command indexercmd = new IndexerCargo();
                new JoystickButton(driverStick, XboxController.Button.kY.value).whenHeld(indexercmd);

                //Command indexerOne = new IndexerOneCargo();
                //new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(indexerOne);
                //Turn a specific angle
                // moveToOuterPort = new TurnToOuterPortCommand(true, 3.0, 0.5);
                // new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(moveToOuterPort, true);

                // Command alignment = new DrivetrainAlignment();
                // new JoystickButton(driverStick, XboxController.Button.kB.value).whenHeld(alignment);

                break;
            }
            case 3: //Cosmobot
            {
                Command setBling =  new SetBlingPattern(2);
                new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(setBling);

                break;
            }
            default:
                break;
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        //Note: if there is not selector,
        //      selectorOne will be forced to selectHardCodedPath
        int selectHardCodedPath = 1;
        int selectorOne = 0;

        if (analogSelectorOne != null){
            selectorOne = analogSelectorOne.getIndex();
            System.out.println("SELECTOR SWITCH NOT NULL AND ID " + selectorOne);
        }
        else
        {
            selectorOne = selectHardCodedPath;
        }
        logger.info("Selectors: " + selectorOne);

        // Testing forced numbers
        int selectFolder = 5;
        switch (selectFolder) {
            case 1:
                return AutoRoutines.getAutoCommandRapidReact(selectorOne); 

            case 2:
                return AutoRoutines.getAutoCommandTest(selectorOne);
                
            case 3:
                return AutoRoutines.getAutoCommandBeetle(selectorOne);
                
            default: 
                return null;
        }
    }
    
    public void joystickRumble(double leftValue, double rightValue) {
        //Joystick rumble (driver feedback). leftValue/rightValue sets vibration force.
        driverStick.setRumble(RumbleType.kLeftRumble, leftValue);
        driverStick.setRumble(RumbleType.kRightRumble, rightValue);
    }

    /**
     * Initialize the current RobotContainer instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new RobotContainer();
        }
    }

    public static RobotContainer getInstance() {
        init();
        return currentInstance;
    }
    
    
}