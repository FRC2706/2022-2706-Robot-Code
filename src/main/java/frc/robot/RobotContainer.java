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
import edu.wpi.first.networktables.NetworkTable;
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
  private Command driveCommand;
  private Command sensitiveDriving;

  private AnalogSelectorSubsystem analogSelectorOne;
  private AnalogSelectorSubsystem analogSelectorTwo;
 
  private Command intakeDown;
  private Command intakeUp;

  private Command cmdIntakeOneCargo;
  private Command cmdIntakeTwoCargo;
  private Command cmdShoot;
  private Command cmdIndexerForShooter;
  private Command cmdIndexerForIntake;

  private Command cmdDriveTrainAlignment;
  private Command cmdTurnToOuterPort;
 
  
  private Logger logger = Logger.getLogger("RobotContainer");
  private final double AUTO_DRIVE_TIME = 1.0;
  private final double AUTO_LEFT_MOTOR_SPEED = 0.2;
  private final double AUTO_RIGHT_MOTOR_SPEED = 0.2;
  

  NetworkTable selectorTable = NetworkTableInstance.getDefault().getTable("selectorTable");
  private NetworkTableEntry tableAnalogSelectorOne, tableAnalogSelectorTwo;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
       
        // Configure the button bindings
        logger.addHandler(Config.logFileHandler);

        configureButtonBindings();

        // Only construct the RelaySubsystem if it has relays which is only on Beetle
        // Beetle's robot id = 2
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
      
        driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y, Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS, true);
        DriveBaseHolder.getInstance().setDefaultCommand(driveCommand);

        //???
        sensitiveDriving = new SensitiveDriverControl(driverStick);
        new JoystickButton(driverStick, XboxController.Button.kLeftBumper.value).whenHeld(sensitiveDriving);
 
        // Command resetHeading = new InstantCommand(() -> DriveBaseHolder.getInstance().resetHeading(Rotation2d.fromDegrees(0)));
        // new JoystickButton(driverStick, XboxController.Button.kBack.value).whenActive(resetHeading);
     
        switch ( Config.robotId )
        {
            case 0: 
            {
                break;
            }
 
            case 1:
            {            
            //============
            //intake
            //=========
            //intake up
            intakeUp = new IntakeUp(); 
            new JoystickButton(controlStick, XboxController.Button.kRightBumper.value).whenReleased(intakeUp);
                
            //intake down
            Command intakeDownFloat = new SequentialCommandGroup(
                                         new ParallelRaceGroup(new IntakeDown(), new WaitCommand(0.5)),
                                         new ParallelRaceGroup(new IntakeFloat(), new WaitCommand(0.5)),
                                         new ParallelCommandGroup(new IndexerOneCargo(), new RunIntakeCargo(true, 0))); 
            new JoystickButton(controlStick, XboxController.Button.kRightBumper.value).whenPressed(intakeDownFloat);
 
            //intake reverse (use it only when it is needed.)
            Command intakeReverse = new RunIntakeCargo(false, 0);
            new JoystickButton(controlStick, XboxController.Button.kBack.value).whenHeld(intakeReverse);

            //=============
            //shooter
            //=================================
            //@todo #4: for shooter command
            //Auto mode backup out of tarmac: target RPM: 3400 RPM
            //at the top of tarmac: 3350 RPM
            //
            //with kicker on: 2550, high goal
            //without kicker on: low goal 1850
            //tarmat A: closer: RMP = 3200
            //no kicker
            //low goal inside tarmac
            Command wait1sA = new WaitCommand(0.5);
            Command delayIndexerA = wait1sA.andThen( new IndexerForShooter());
            Command cmdShootA = new ParallelCommandGroup(new SpinUpShooterWithTime(1800, 0), delayIndexerA);
            Command kickerDownA = new ParallelRaceGroup(new ControlKicker(false), new WaitCommand(0.5));
            Command kickerShootA = new SequentialCommandGroup(kickerDownA, cmdShootA);
            new JoystickButton(controlStick, XboxController.Button.kX.value).whenHeld(kickerShootA);
            //.whenHeld(cmdShootA);
            //left trigger: 2
            //right trigger: 3
            //if(controlStick.getRawAxis(2) > 0.5);
            //new JoystickButton(controlStick, XboxController.Axis.kRightTrigger.value);


            // //tarmat B: farther: RPM = 3400
            //no kicker
            //high goal position B
            Command wait1sB = new WaitCommand(0.5);
            Command delayIndexerB = wait1sB.andThen( new IndexerForShooter());
            Command cmdShootB = new ParallelCommandGroup(new SpinUpShooterWithTime(2920, 0), delayIndexerB);
            Command kickerDownB = new ParallelRaceGroup(new ControlKicker(false), new WaitCommand(0.5));
            Command kickerShootB = new SequentialCommandGroup(kickerDownB, cmdShootB);
            new JoystickButton(controlStick, XboxController.Button.kB.value).whenHeld(kickerShootB);

            //high goal position C
            Command wait1sC= new WaitCommand(0.5);
            Command delayIndexerC = wait1sC.andThen( new IndexerForShooter());
            Command cmdShootC = new ParallelCommandGroup(new SpinUpShooterWithTime(3050, 0), delayIndexerC);
            Command kickerDownC = new ParallelRaceGroup(new ControlKicker(false), new WaitCommand(0.5));
            Command kickerShootC = new SequentialCommandGroup(kickerDownC,cmdShootC);
            new JoystickButton(controlStick, XboxController.Button.kA.value).whenHeld(kickerShootC);

            //kicker floating
            Command kickerFloat = new KickerFloat();
            new JoystickButton(controlStick, XboxController.Button.kStart.value).whenPressed(kickerFloat);
 
            //test switch
            // Command readSwitch = new TestSwitch();
            // new JoystickButton(driverStick, XboxController.Button.kStart.value).whenHeld(readSwitch);

            //Command RPMClimb = new ClimberRPM();
            //new JoystickButton(controlStick,XboxController.Axis.kRightTrigger.value).whenHeld(RPMClimb);

            // Command climbTrigger = new ClimberRPM(() -> controlStick.getRawAxis(XboxController.Axis.kLeftTrigger.value));
            Command slowClimb = new ClimberRPM(0.1);
            Command fastClimb = new ClimberRPM(0.8);
            
            new JoystickButton(controlStick, XboxController.Button.kLeftStick.value).whenHeld(slowClimb);
            new JoystickButton(controlStick, XboxController.Button.kRightStick.value).whenHeld(fastClimb);
              
            //Command positionClimb = new ClimberPosition();
            //new JoystickButton(controlStick, XboxController.Axis.kRightTrigger.value).whenPressed(positionClimb);

            break;
            }
           case 2: //Beetle
            {
                //Front ring light
                //Command controlFrontRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_FRONT);
                //new JoystickButton(driverStick, XboxController.Button.kRightBumper.value).whenPressed(controlFrontRinglight);
                
                //Rear small ring light
               // Command controlRearSmallRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_REAR_SMALL);
               // new JoystickButton(driverStick, XboxController.Button.kX.value).whenPressed(controlRearSmallRinglight);
                
                //Command printX = new PrintOdometry();
                //new JoystickButton(driverStick, XboxController.Button.kX.value).whenPressed(printX);

                //Rear large ring light
                Command controlRearLargeRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_REAR_LARGE);
                new JoystickButton(controlStick, XboxController.Button.kRightBumper.value).whenPressed(controlRearLargeRinglight);
                        
                //Read a trajectory
                // Command readTrajectory = new ReadPath( Robot.trajectoryRead, "Slalom path");
                // new JoystickButton(driverStick, XboxController.Button.kB.value).whenPressed(readTrajectory);

                // Command readIrSensor = new ReadAnalogInput(7);
                // new JoystickButton(driverStick, XboxController.Button.kA.value).whenPressed(readIrSensor);

               Command readSwitch = new TestSwitch();
             new JoystickButton(driverStick, XboxController.Button.kStart.value).whenHeld(readSwitch);

            //    Command readColorSensor = new ReadColorSensor();
             //   new JoystickButton(driverStick, XboxController.Button.kB.value).whenPressed(readColorSensor);

                //for shooter command
                //Auto mode backup out of tarmac: target RPM: 3400 RPM
                //at the top of tarmac: 3350 RPM
                Command wait1s = new WaitCommand(1);
                Command delayIndexer = wait1s.andThen( new IndexerForShooter());
                Command shooter = new ParallelCommandGroup(new SpinUpShooterWithTime(3350, 0), delayIndexer);
                new JoystickButton(driverStick, XboxController.Button.kB.value).whenHeld(shooter);
                
                //Command testIndexer = new TestIndexer();
                //new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(testIndexer);

                //indexer cargo command
                 Command indexercmd = new IndexerCargo();
                 new JoystickButton(driverStick, XboxController.Button.kY.value).whenHeld(indexercmd);

                // Command testAnalog = new TestAnalogSelector();
                // new JoystickButton(driverStick, XboxController.Button.kA.value).whenPressed(testAnalog);
                Command indexerOne = new IndexerOneCargo();
                new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(indexerOne);
                //Turn a specific angle
                // moveToOuterPort = new TurnToOuterPortCommand(true, 3.0, 0.5);
                // new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(moveToOuterPort, true);

               // Command alignment = new DrivetrainAlignment(false);
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

        tableAnalogSelectorOne = selectorTable.getEntry("AnalogSelectorOne");
        tableAnalogSelectorTwo = selectorTable.getEntry("AnalogSelectorTwo");

        analogSelectorOne = AnalogSelectorSubsystem.getInstance();
        
        if (analogSelectorOne != null){
            selectorOne = analogSelectorOne.getIndex();
            System.out.println("SELECTOR SWITCH NOT NULL AND ID " + selectorOne);
            tableAnalogSelectorOne.setValue(selectorOne);
        }
        else
        {
            selectorOne = selectHardCodedPath;
        }
        
        logger.info("Selectors: " + selectorOne);


        // Testing forced numbers
        int selectFolder = 1;
        //@todo: hard coded here. Remove this line will use analog selector.
        //selectorOne = 0;
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
