package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Robot;
import frc.robot.commands.ramseteAuto.VisionPose;
import frc.robot.commands.ramseteAuto.VisionPose.VisionType;
import frc.robot.config.Config;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.logging.Logger;

import frc.robot.commands.ramseteAuto.SimpleCsvLogger;

public class DriveBaseNeo extends DriveBase {
    private CANSparkMax leftMaster, rightMaster;    
    private CANSparkMax leftFollower, rightFollower;

    private SparkMaxPIDController m_pidControllerLeftMaster;
    private SparkMaxPIDController m_pidControllerRightMaster;
    private RelativeEncoder m_encoderLeftMaster;
    private RelativeEncoder m_encoderRightMaster;

    public double kMaxOutput = 1;
    public double kMinOutput = -1;
    
    private DifferentialDriveOdometry odometry;    

    public double motorCurrent; //variable to display motor current levels
    public boolean motorLimitActive = false; //states if motor current is actively being limited
    
    // Logging
    private Logger logger = Logger.getLogger("DriveBase2020");

    // USB Logger
    private boolean bUsbLogger = false;
    private SimpleCsvLogger usbLogger;
    private String loggingDataIdentifier = "DriveBase2020";

    private NetworkTableEntry leftEncoder, rightEncoder, currentX, currentY, currentAngle, currentPose, leftVelocityMetersPerSecond, rightVelocityMetersPerSecond;
    public DriveBaseNeo() {
    
        if ( Config.LEFT_FRONT_MOTOR != -1)
        {
            leftMaster  = new CANSparkMax(Config.LEFT_FRONT_MOTOR, MotorType.kBrushless);
        }
        else
        {
            leftMaster = null;
        }

        if ( Config.RIGHT_FRONT_MOTOR != -1)
        {
            rightMaster = new CANSparkMax(Config.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
        }

        if(Config.HAS_FOLLOWERS == true)
        {
            if (Config.LEFT_REAR_MOTOR != -1 )
            {
                leftFollower = new CANSparkMax(Config.LEFT_REAR_MOTOR, MotorType.kBrushless);
            }
            else
            {
                leftFollower = null;
            }
        
            if (Config.RIGHT_REAR_MOTOR != -1 )
            {
                rightFollower = new CANSparkMax(Config.RIGHT_REAR_MOTOR, MotorType.kBrushless);
            }
            else
            {
                rightFollower = null;
            }
        }

        if ( leftMaster == null || rightMaster == null )
        {
            state = DriveBaseState.Degraded;
            return;
        }

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getCurrentAngle()));        

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        rightMaster.setInverted(true);

        resetMotors();
        setMotorConfigurations();
        setCoastMode();

        if (Config.PIGEON_ID != -1) 
        {
            pigeon = new PigeonIMU(Config.PIGEON_ID);
        }
        else
        {
            pigeon = null;
        }


        if ( pigeon != null)
        {
            pigeon.setFusedHeading(0d, Config.CAN_TIMEOUT_LONG);
        }
        else
        {
            state = DriveBaseState.Degraded;
        }         

        logger.addHandler(Config.logFileHandler);

        var table = NetworkTableInstance.getDefault().getTable("DrivetrainData");
        leftEncoder = table.getEntry("leftEncoder");
        rightEncoder = table.getEntry("rightEncoder");
        currentX = table.getEntry("currentX");
        currentY = table.getEntry("currentY");
        currentAngle = table.getEntry("currentAngle");
        currentPose = table.getEntry("currentPose");
        leftVelocityMetersPerSecond = table.getEntry("leftVelocityMetersPerSecond");
        rightVelocityMetersPerSecond = table.getEntry("rightVelocityMetersPerSecond");
    
        if ( bUsbLogger == true )
        {
          usbLogger = new SimpleCsvLogger();
        }
        else
        {
          usbLogger = null;
        }
    }    

    @Override
    public double getMotorCurrent() {
        //Get motor supply current, send it to shuffleboard, and return it.
        motorCurrent = (leftMaster.getOutputCurrent() + rightMaster.getOutputCurrent())/2;
        SmartDashboard.putNumber("Avg Motor Current", motorCurrent);
        return(motorCurrent); //Returns average motor current draw.
    }

    @Override
    public boolean isMotorLimitActive() {
        //Checks if motor currents are at or above the continuous limit (checks if current limiting is imminent or ongoing)
        //This method does not limit motor current. It monitors current for driver feedback purposes.
        if (((leftMaster.getOutputCurrent() >= Config.CONTIN_CURRENT_AMPS) == true) 
             || ((rightMaster.getOutputCurrent() >= Config.CONTIN_CURRENT_AMPS) == true)) {
            motorLimitActive = true;
        }
        else {
            motorLimitActive = false;
        }

        //Tell shuffleboard if current limting is active and return the result.
        SmartDashboard.putBoolean("MotorCurrentLimit T/F", motorLimitActive);
        return(motorLimitActive);
    }

    @Override
    public void stopMotors() {
        if ( leftMaster != null )
            leftMaster.stopMotor();

        if ( rightMaster != null )
            rightMaster.stopMotor();

        if(leftFollower != null){
            leftFollower.stopMotor();
        }
        if(rightFollower != null){
            rightFollower.stopMotor();
        }
    }
    
    @Override
    protected void resetMotors() {
        
        REVLibError errCode = leftMaster.restoreFactoryDefaults();
        if ( errCode != REVLibError.kOk )
        {
            state = DriveBaseState.Degraded;
        }

        errCode =  rightMaster.restoreFactoryDefaults();
        if ( errCode != REVLibError.kOk )
        {
           state = DriveBaseState.Degraded;
        }    

        if(leftFollower != null)
        {
            errCode = leftFollower.restoreFactoryDefaults();
            if ( errCode != REVLibError.kOk )
            {
                state = DriveBaseState.Degraded;
            }   
        }

        if(rightFollower != null)
        {
            errCode = rightFollower.restoreFactoryDefaults();
            if ( errCode != REVLibError.kOk )
            {
                state = DriveBaseState.Degraded;
            }
    
        }

        errCode = leftMaster.setSmartCurrentLimit(60);
        if ( errCode != REVLibError.kOk )
        {
            state = DriveBaseState.Degraded;
        }

        errCode = rightMaster.setSmartCurrentLimit(60);
        if ( errCode != REVLibError.kOk )
        {
            state = DriveBaseState.Degraded;
        }

        this.followMotors();
    }

    private void setMotorConfigurations() {

        leftMaster.setInverted(Config.LEFT_FRONT_INVERTED);
        rightMaster.setInverted(Config.RIGHT_FRONT_INVERTED);
        if(leftFollower != null){
            leftFollower.setInverted(Config.LEFT_REAR_INVERTED);
        }
        if(rightFollower != null){
            rightFollower.setInverted(Config.RIGHT_REAR_INVERTED);
        }    

        //PID controller for the shooter
        m_pidControllerLeftMaster = leftMaster.getPIDController();
        m_encoderLeftMaster = leftMaster.getEncoder();

        m_pidControllerRightMaster = rightMaster.getPIDController();
        m_encoderRightMaster = rightMaster.getEncoder();

        //left master: slot 1 belongs to Ramsete: velocity control
        REVLibError errorCode;
        errorCode = m_pidControllerLeftMaster.setOutputRange(kMinOutput, kMaxOutput, 1);
        errorCode = m_pidControllerLeftMaster.setFF(0, 1); //Config.RAMSETE_KF;
        errorCode = m_pidControllerLeftMaster.setP(0, 1); //Config.RAMSETE_KP;
        errorCode = m_pidControllerLeftMaster.setI(0, 1); //Config.RAMSETE_KI;
        errorCode = m_pidControllerLeftMaster.setD(0, 1); //Config.RAMSETE_KD;
        errorCode = m_pidControllerLeftMaster.setIZone(100, 1);
      
        //right master: slot 1 belongs to Ramsete
        errorCode = m_pidControllerRightMaster.setOutputRange(kMinOutput, kMaxOutput, 1);
        errorCode = m_pidControllerRightMaster.setFF(0, 1); //Config.RAMSETE_KF;
        errorCode = m_pidControllerRightMaster.setP(0, 1); //Config.RAMSETE_KP;
        errorCode = m_pidControllerRightMaster.setI(0, 1); //Config.RAMSETE_KI;
        errorCode = m_pidControllerRightMaster.setD(0, 1); //Config.RAMSETE_KD;
        errorCode = m_pidControllerRightMaster.setIZone(100, 1);

        leftMaster.enableVoltageCompensation(Config.RAMSETE_VOLTAGE_COMPENSATION);
        rightMaster.enableVoltageCompensation(Config.RAMSETE_VOLTAGE_COMPENSATION);

        //left master: Slot 2 belongs to drive train alignment: position control
        errorCode = m_pidControllerLeftMaster.setOutputRange(kMinOutput, kMaxOutput, 2);
        errorCode = m_pidControllerLeftMaster.setFF(0, 2); //Config.ALIGNMENT_KF;
        errorCode = m_pidControllerLeftMaster.setP(0, 2);  //Config.ALIGNMENT_KP;
        errorCode = m_pidControllerLeftMaster.setI(0, 2);  //Config.ALIGNMENT_KI;
        errorCode = m_pidControllerLeftMaster.setD(0, 2);  //Config.ALIGNMENT_KD;
        errorCode = m_pidControllerLeftMaster.setIZone(100, 2);
        
        //Set max acceleration, velocity, and minimum velocity
        errorCode = m_pidControllerLeftMaster.setSmartMotionMaxAccel(0, 2);
        errorCode = m_pidControllerLeftMaster.setSmartMotionMaxVelocity(0, 2);      
        // the following line cause the trouble: return error code not kOk
        //  errorCode = m_pidControllerLeftMaster.setSmartMotionMinOutputVelocity(-1, 2);      

        errorCode = m_pidControllerRightMaster.setOutputRange(kMinOutput, kMaxOutput, 2);
        errorCode = m_pidControllerRightMaster.setFF(0, 2); //Config.ALIGNMENT_KF;
        errorCode = m_pidControllerRightMaster.setP(0, 2);  //Config.ALIGNMENT_KP;
        errorCode = m_pidControllerRightMaster.setI(0, 2);  //Config.ALIGNMENT_KI;
        errorCode = m_pidControllerRightMaster.setD(0, 2);  //Config.ALIGNMENT_KD;
        errorCode = m_pidControllerRightMaster.setIZone(100, 2);
     
        //Set max acceleration, velocity, and minimum velocity
        errorCode = m_pidControllerRightMaster.setSmartMotionMaxAccel(0, 2);
        errorCode = m_pidControllerRightMaster.setSmartMotionMaxVelocity(0, 2);      
        // the following line cause the trouble: return error code not kOk
        //  errorCode = m_pidControllerRightMaster.setSmartMotionMinOutputVelocity(-1, 2);     
         

       //@todo: for sparkmax
        //talonConfig.neutralDeadband = Config.DRIVE_OPEN_LOOP_DEADBAND;
        //talonConfig.slot1.allowableClosedloopError = Config.RAMSETE_ALLOWABLE_PID_ERROR;

    //    if (!leftMasterError.equals(ErrorCode.OK)) 
    //     {
    //         logErrorCode(leftMasterError, "DrivetrainLeftMaster", Config.LEFT_FRONT_MOTOR, "configAllSettings");
    //         state = DriveBaseState.Degraded;
    //         System.out.println("DriveBase2020 Degraded: Line 221");
    //     }
    //     if (!rightMasterError.equals(ErrorCode.OK))
    //     {
    //         logErrorCode(rightMasterError, "DrivetrainRightMaster", Config.RIGHT_FRONT_MOTOR, "configAllSettings");
    //         state = DriveBaseState.Degraded;
    //         System.out.println("DriveBase2020 Degraded: Line 227");
    //     }

        m_encoderLeftMaster = leftMaster.getEncoder();
        m_encoderRightMaster = rightMaster.getEncoder();

        if ( m_encoderLeftMaster == null || m_encoderRightMaster == null )
        {
            //error case
        }

        // Config the encoder and check if it worked
        
        // if (e1.value != 0 || e2.value != 0) {
        //     state = DriveBaseState.Degraded;
        //     System.out.println("DriveBase2020 Degraded: Line 236");
        //     logger.severe("DRIVETRAIN ENCODER NOT WORKING - DRIVETRAIN DEGRADED - ONLY DRIVER CONTROLS ACTIVE");
        //     logErrorCode(e1, "DrivetrainLeftMaster", Config.LEFT_FRONT_MOTOR, "configSelectedFeedbackSensor(MagEncoderRelative)");
        //     logErrorCode(e2, "DrivetrainRightMaster", Config.RIGHT_FRONT_MOTOR, "configSelectedFeedbackSensor(MagEncoderRelative)");
        // }
        
        // set the encoder inversions
        // leftMaster.setSensorPhase(Config.DRIVETRAIN_LEFT_SENSORPHASE);
        // rightMaster.setSensorPhase(Config.DRIVETRAIN_RIGHT_SENSORPHASE);

        // e1 = leftMaster.setSelectedSensorPosition(0);
        // e2 = rightMaster.setSelectedSensorPosition(0);

        // if (e1.value != 0 || e2.value != 0) {
        //     state = DriveBaseState.Degraded;
        //     System.out.println("DriveBase2020 Degraded: Line 265");
        // }

    }

    /**
     * Log error code will take a CTRE ErrorCode object.
     * 
     * It does nothing if the error code is ok
     * 
     * It will log if the error code is bad. It shows the ErrorCode
     * name, number and category.
     * 
     * @param e The error code
     * @param motorName A name to define which device this code belongs to
     * @param canID the can id belonging to the device
     * @param action Name of the method that produced this error
     */
    private void logErrorCode(ErrorCode e, String motorName, int canID, String methodName) {
        if (e.equals(ErrorCode.OK)) {
            return;
        }

        String[] errorCategories = new String[]{"CAN-Related", "UserSpecifiedGeneral", "Signal", "Gadgeteer Port Error Codes", 
                    "Gadgeteer Module Error Codes", "API", "Higher Level", "CAN Related", "General", "Simulation"};
        String errorCategory;
        if (e.value >= -8 && e.value <= 10) 
            errorCategory = errorCategories[0];
        else if(e.value == -100) 
            errorCategory = errorCategories[1];
        else if(e.value == -200 || e.value == -201) 
            errorCategory = errorCategories[2];
        else if(e.value == -300 || e.value == -301)
            errorCategory = errorCategories[3];
        else if(e.value == -400 || e.value == -401 || e.value == -402)
            errorCategory = errorCategories[4];
        else if(e.value >= -505 && e.value <= -500)
            errorCategory = errorCategories[5];
        else if(e.value == -600 || e.value == -601)
            errorCategory = errorCategories[6];
        // skip errorCategories[7] b/c its included in errorCategories[0]
        else if(e.value >= 100 && e.value <= 110)
            errorCategory = errorCategories[8];
        else if(e.value == 200 || e.value == 201 || e.value == 202)
            errorCategory = errorCategories[9];
        else
            errorCategory = "Unknown Category";

        String logString = String.format("MOTOR: %s, CANID: %d, ERROR NAME: %s, ERROR CATEGORY: %s, PRODUCED BY METHOD: %s", motorName, canID, e.name(), errorCategory, methodName);

        // Log the error code as severe
        logger.severe("CTRE ErrorCode - " + logString); 
        
    }

    @Override
    public void setCoastMode() {
        leftMaster.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);

        if(leftFollower != null){
            leftFollower.setIdleMode(IdleMode.kCoast);
        }
        if(rightFollower != null){
            rightFollower.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public void setBrakeMode() {
        leftMaster.setIdleMode(IdleMode.kBrake);
        rightMaster.setIdleMode(IdleMode.kBrake);

        if(leftFollower != null){
            leftFollower.setIdleMode(IdleMode.kBrake);
        }
        if(rightFollower != null){
            rightFollower.setIdleMode(IdleMode.kBrake);
        }
    }
    
    @Override
    protected void followMotors() {
        if(leftFollower != null){
            leftFollower.follow(leftMaster);
        }
        if(rightFollower != null){
            rightFollower.follow(rightMaster);
        }
    }
    
    @Override
    protected void driveModeUpdated(DriveMode mode) {
        
        if (mode == DriveMode.OpenLoopVoltage) {
            setActivePIDSlot(Config.DRIVETRAIN_SLOTID_DRIVER);
            
        } else if (mode == DriveMode.Disabled) {
            stopMotors();
        }
    }

    @Override
    public void periodic() {
        if (hasPigeon()) {

            odometry.update(Rotation2d.fromDegrees(getCurrentAngle()), getLeftPosition(), getRightPosition());
        
            //@todo: try
            //odometry.update(Rotation2d.fromDegrees(-getCurrentAngle()), getLeftPosition(), getRightPosition());
   
            //from encoders' positions
            leftEncoder.setNumber(getLeftPosition());
            rightEncoder.setNumber(getRightPosition());

            //pose from odometry
            Pose2d pose = getPose();
            currentX.setNumber(pose.getX());
            currentY.setNumber(pose.getY());
            currentAngle.setNumber(pose.getRotation().getDegrees());

            currentPose.setString(String.format("new PoseScaled(%.3f, %.3f, %.3f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
            double measuredVelocities[] = getMeasuredMetersPerSecond();

            leftVelocityMetersPerSecond.setNumber(measuredVelocities[0]);
            rightVelocityMetersPerSecond.setNumber(measuredVelocities[1]);

            logData(getLeftPosition(),
                    getRightPosition(),
                    pose.getX(),
                    pose.getY(),
                    pose.getRotation().getDegrees(),
                    measuredVelocities[0],
                    measuredVelocities[1]);  
        }

        // System.out.println("VISION TARGET: " + VisionPose.getInstance().getTargetTranslation(VisionType.TPracticeTarget)); 
    }

    /**
     * Returns the a Pose2d of the current robot location
     * 
     * Odometry calcules a new pose every robot cycle and stores
     * the value so this method is only reading the stored value.
     * This means we only do 1 hardware read every cycle instead of 
     * many things calling hardware redunantly
     * 
     * @param Pose2d the current pose of the robot
     */
    @Override 
    public Pose2d getPose() { 
        return odometry.getPoseMeters();
    }

    /**
     * This method will return the heading from odometry
     * 
     * Odometry keeps track of the gyro heading and in relation to
     * the value it was reset to using an offset so it's important to ask
     * the odometry for the rotation instead of directly from the gyro.
     * 
     * @param Rotation2d The heading. Rotation2d has a .getDegrees() method.
     */
    @Override
    public Rotation2d getOdometryHeading() {
        return getPose().getRotation();
    }

    @Override
    public void resetPose(Pose2d newPose) {
        REVLibError leftError = m_encoderLeftMaster.setPosition(0);
        REVLibError rightError = m_encoderRightMaster.setPosition(0);
            
        //logErrorCode(leftError, "DrivetrainLeftMaster", Config.LEFT_FRONT_MOTOR, "setSelectedSensorPosition(0)");
        //logErrorCode(rightError, "DrivetrainRighttMaster", Config.RIGHT_FRONT_MOTOR, "setSelectedSensorPosition(0)");

        odometry.resetPosition(newPose, Rotation2d.fromDegrees(getCurrentAngle()));
    }

    /**
     * Resets the heading of the robot to a desired value
     * 
     * To construct a Rotation2d from degrees, use Rotation2d.fromDegrees(deg)
     * Otherwise the constructor uses radians, new Rotation2d(rad)
     * 
     * If also changing odometry x and y, just use resetPose
     * 
     * @param newHeading for degrees, do resetHeading(Rotation2d.fromDegrees(deg))
     */
    @Override
    public void resetHeading(Rotation2d newHeading) {
        Translation2d currentTranslation = getPose().getTranslation();
        resetPose(new Pose2d(currentTranslation, newHeading));
    }

    @Override
    public void setActivePIDSlot(int slotId) {
        //don't need this for sparkmax, since it pidController has slot id as input
        // leftMaster.selectProfileSlot(slotId, Config.TALON_PRIMARY_PID);
        // rightMaster.selectProfileSlot(slotId, Config.TALON_PRIMARY_PID);
    }

    @Override
    public void tankDriveVelocities(double leftVel, double rightVel, double leftFF, double rightFF) {

        //slot 1: for ramsete
        //@todo: metersPerSecondToRPM
        m_pidControllerLeftMaster.setReference(1000, 
            ControlType.kVelocity, 1, leftFF / 12.0, com.revrobotics.SparkMaxPIDController.ArbFFUnits.kVoltage);

        m_pidControllerRightMaster.setReference(1000, 
            ControlType.kVelocity, 1, rightFF / 12.0, com.revrobotics.SparkMaxPIDController.ArbFFUnits.kVoltage);

        // leftMaster.set(ControlMode.Velocity, metersPerSecondToTalonVelocity(leftVel), 
        //         DemandType.ArbitraryFeedForward, leftFF / 12.0);

        // rightMaster.set(ControlMode.Velocity, metersPerSecondToTalonVelocity(rightVel), 
        //         DemandType.ArbitraryFeedForward, rightFF / 12.0); 

        differentialDrive.feed();
    }

    /**
     * Drive a specific distance using Motion magic position control
     * @param leftPos target position on the left side
     * @param rightPos target position on the right side
     */
    @Override
    public void tankDrivePosition( double leftPos, double rightPos)
    {
        //slot 2: alignment

        //@todo: double check get the current position

        //smart position close-loop control
        //@todo: metersToSparkMaxPosition
        m_pidControllerLeftMaster.setReference(leftPos, ControlType.kSmartMotion, 2);
        m_pidControllerRightMaster.setReference(rightPos, ControlType.kSmartMotion, 2);
        
        differentialDrive.feed();
    }

    /**
     * Obtain the measured raw velocities on both sides
     * @return double array
     */
    @Override
    public double[] getMeasuredVelocities() {
        double leftVel = m_encoderLeftMaster.getVelocity(); //unit: RPM
        double rightVel = m_encoderRightMaster.getVelocity(); //unit: RPM
        return new double[]{leftVel, rightVel};
    }

    /**
     * Obtain the measured velocity in m/s on both sides
     * @return double array
     */
    @Override
    public double[] getMeasuredMetersPerSecond() {
        double[] velRPMUnits = getMeasuredVelocities();
        double leftVel = RPMToMetersPerSecond(velRPMUnits[0]);
        double rightVel = RPMToMetersPerSecond(velRPMUnits[1]);
        return new double[]{leftVel, rightVel};
    }

    /**
     * Obtain left encoder position in meters
     * @return double
     */
    @Override
    public double getLeftPosition() {
        return sparkPosistionToMeters(m_encoderLeftMaster.getPosition());
    }

    /**
     * Obtain right encoder position in meters
     * @return double
     */
    @Override
    public double getRightPosition() {
        return sparkPosistionToMeters(m_encoderLeftMaster.getPosition());
    }

    /**
     * Obtain left encoder position in raw unit: # rotations
     * @return double
     */
    @Override
    public double getLeftEncoderPosition() {
        return m_encoderLeftMaster.getPosition();
    }

    /**
     * Obtain right encoder position in raw unit: # rotations
     * @return double
     */
    @Override
    public double getRightEncoderPosition() {
        return m_encoderRightMaster.getPosition();
    }

    /**
     * Converting Talon ticks to meters
     * 
     * Unit Conversion Method
     */
    private double talonPositionToMeters(double talonPosisiton) {
        double result = talonPosisiton;
        double circumference = Math.PI * Config.drivetrainWheelDiameter;
        double metersPerTick = circumference / Config.ticksPerRevolution;
        result *= metersPerTick;
        return result;  
    }

    /**
     * Converting m/s to talon ticks/100ms
     *  
     * Unit Conversion Method
     */
    private double metersPerSecondToTalonVelocity(double metersPerSecond) {
        return metersToTalonPosistion(metersPerSecond * 0.1); // Converting meters per second to meters per 100ms
    }

    /**
     * Converting meters to talon ticks
     * 
     * Unit Conversion Method
     */
    private double metersToTalonPosistion(double meters) {
        double result = meters;
        double circumference = Math.PI * Config.drivetrainWheelDiameter; // Pi*Diameter
        double ticksPerMeter = Config.ticksPerRevolution / circumference; // Ticks per revolution / circumference
        result = result * ticksPerMeter; // Meter * ticks in 1 meter
        return result;
    }

    /**
     * Converting Talon ticks to m/s
     * 
     * Unit Conversion Method
     */
    private double talonPosistionToMeters(double talonPosisiton) {
        double result = talonPosisiton;
        double circumference = Math.PI * Config.drivetrainWheelDiameter;
        double metersPerTick = circumference / Config.ticksPerRevolution;
        result *= metersPerTick;
        return result;

    }

    private double sparkPosistionToMeters(double sparkPosisiton) {
        double result = sparkPosisiton;
        double circumference = Math.PI * Config.drivetrainWheelDiameter;
        result = sparkPosisiton * circumference;
        return result;

    }

    /**
     * Converting talon ticks/100ms to m/s
     * 
     * Unit Conversion Method
     */
    private double talonVelocityToMetersPerSecond(double talonVelocity) {
        return talonPosistionToMeters(talonVelocity * 10); // Convert ticks/100ms to ticks/sec
    }

    /**
     * Converting spark max RPM (rotation per minute) to m/s
     * 
     * Unit Conversion Method
     */
    private double RPMToMetersPerSecond(double RPM) {
        return (RPM * Math.PI * Config.drivetrainWheelDiameter /60); 
    }

    public DriveBaseState getDriveBaseState()
    {
        return state;
    }
    /**
     * All Methods used for USB logging startLogging() logData(data) stopLogging()
     */
    @Override
    public void startLogging() {
        if ( usbLogger != null )
        {
        // See Spreadsheet link at top
            usbLogger.init(loggingDataIdentifier, 
                    new String[] { "LPos",
                                "RPos",
                                "currentX",
                                "currentY",
                                "currentAngle",
                                "LVel",
                                "RVel"},
                    new String[] { "m", 
                                "m",
                                "m",
                                "m",
                                "deg",
                                "m/s",
                                "m/s"});
        }
    }


    public void logData(double... data) {
        if ( usbLogger != null )
        {
            usbLogger.writeData(data);
        }
    }

    @Override
    public void stopLogging() {
        if ( usbLogger != null )
        {
            usbLogger.close();
        }
    }
}

