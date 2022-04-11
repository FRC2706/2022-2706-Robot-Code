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

public class DriveBase2020 extends DriveBase {
    WPI_TalonSRX leftMaster, rightMaster;    
    BaseMotorController leftFollower, rightFollower;
    
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
    public DriveBase2020() {
        leftMaster = new WPI_TalonSRX(Config.LEFT_MASTER);
        rightMaster = new WPI_TalonSRX(Config.RIGHT_MASTER);

        // Check whether to construct a victor or a talon or nothing
        if(Config.HAS_FOLLOWERS == true){
            if (Config.LEFT_SLAVE_ISVICTOR) {
                leftFollower = new WPI_VictorSPX(Config.LEFT_REAR_MOTOR);
            } else {
                leftFollower = new WPI_TalonSRX(Config.LEFT_REAR_MOTOR);
            }
            if (Config.RIGHT_SLAVE_ISVICTOR) {
                rightFollower = new WPI_VictorSPX(Config.RIGHT_REAR_MOTOR);
            } else {
                rightFollower = new WPI_TalonSRX(Config.RIGHT_REAR_MOTOR);
            }
        }
        else{
            leftFollower = null;
            rightFollower = null;
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
        setTalonConfigurations();
        setCoastMode();

        if (Config.PIGEON_ID != -1) {
            if (Config.PIGEON_ID == Config.LEFT_REAR_MOTOR && leftFollower != null) 
                pigeon = new PigeonIMU((WPI_TalonSRX) leftFollower);
            else if (Config.PIGEON_ID == 27){
               pigeon = new PigeonIMU(Config.PIGEON_ID);
            }
            else{
                pigeon = new PigeonIMU(new WPI_TalonSRX(Config.PIGEON_ID));
            }

            if ( pigeon != null)
                pigeon.setFusedHeading(0d, Config.CAN_TIMEOUT_LONG);
            else
            {
                state = DriveBaseState.Degraded;
            }
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
        motorCurrent = (leftMaster.getSupplyCurrent() + rightMaster.getSupplyCurrent())/2;
        SmartDashboard.putNumber("Avg Motor Current", motorCurrent);
        return(motorCurrent); //Returns average motor current draw.
    }

    @Override
    public boolean isMotorLimitActive() {
        //Checks if motor currents are at or above the continuous limit (checks if current limiting is imminent or ongoing)
        //This method does not limit motor current. It monitors current for driver feedback purposes.
        if (((leftMaster.getSupplyCurrent() >= Config.CONTIN_CURRENT_AMPS) == true) || ((rightMaster.getSupplyCurrent() >= Config.CONTIN_CURRENT_AMPS) == true)) {
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
            leftFollower.neutralOutput();
        }
        if(rightFollower != null){
            rightFollower.neutralOutput();
        }
    }
    
    @Override
    protected void resetMotors() {
        
        ErrorCode errCode = leftMaster.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

       errCode =  rightMaster.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
       if ( errCode.value != 0 )
       {
           state = DriveBaseState.Degraded;
       }    

        if(leftFollower != null){
            errCode = leftFollower.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
            if ( errCode.value != 0 )
            {
                state = DriveBaseState.Degraded;
            }   
        }

        if(rightFollower != null){
            errCode = rightFollower.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
            if ( errCode.value != 0 )
            {
                state = DriveBaseState.Degraded;
            }
    
        }

        errCode = leftMaster.configPeakCurrentLimit(0);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

        errCode = leftMaster.configPeakCurrentDuration(0);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

        errCode = leftMaster.configContinuousCurrentLimit(0);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

        errCode = rightMaster.configPeakCurrentLimit(0);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

        errCode = rightMaster.configPeakCurrentDuration(0);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

        errCode = rightMaster.configContinuousCurrentLimit(0);
        if ( errCode.value != 0 )
        {
            state = DriveBaseState.Degraded;
        }

        this.followMotors();
    }

    private void setTalonConfigurations() {
        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration(); 

        // Put most settings for talon here
        talonConfig.neutralDeadband = Config.DRIVE_OPEN_LOOP_DEADBAND;

        // Slot 1 belongs to Ramsete
        talonConfig.slot1.kF = Config.RAMSETE_KF;
        talonConfig.slot1.kP = Config.RAMSETE_KP;
        talonConfig.slot1.kI = Config.RAMSETE_KI;
        talonConfig.slot1.kD = Config.RAMSETE_KD;
        talonConfig.slot1.allowableClosedloopError = Config.RAMSETE_ALLOWABLE_PID_ERROR;

        talonConfig.voltageCompSaturation = Config.RAMSETE_VOLTAGE_COMPENSATION;

        // Slot 2 belongs to drive train alignment
        talonConfig.slot2.kF = Config.ALIGNMENT_KF;
        talonConfig.slot2.kP = Config.ALIGNMENT_KP;
        talonConfig.slot2.kI = Config.ALIGNMENT_KI;
        talonConfig.slot2.kD = Config.ALIGNMENT_KD;
        talonConfig.slot2.allowableClosedloopError = Config.ALIGNMENT_ALLOWABLE_PID_ERROR;
        //Motion Magic Closed-Loop Configs
        //tuned for Beetle on competition carpet: Acceleration=0.6, Velocity=0.15, Strength=6.
        talonConfig.motionAcceleration   = metersToTalonPosistion(0.6); //6m/s^2
        talonConfig.motionCruiseVelocity = metersToTalonPosistion(0.15); //1.5 m/sec
        talonConfig.motionCurveStrength  = 6;    
        
        //Current limiting for drivetrain master motors.
        if (Config.MOTOR_CURRENT_LIMIT == true) {
            talonConfig.peakCurrentLimit = Config.PEAK_CURRENT_AMPS;
            talonConfig.peakCurrentDuration = Config.PEAK_TIME_MS;
            talonConfig.continuousCurrentLimit = Config.CONTIN_CURRENT_AMPS;
        } else { 
            //If MOTOR_CURRENT_LIMIT is not true, remove talon current limits, just to be safe.
            talonConfig.peakCurrentLimit = 0;
            talonConfig.peakCurrentDuration = 0;
            talonConfig.continuousCurrentLimit = 0;
        }
        
        // Config all talon settings - automatically returns worst error
        ErrorCode leftMasterError = leftMaster.configAllSettings(talonConfig);
        ErrorCode rightMasterError = rightMaster.configAllSettings(talonConfig);

        if (!leftMasterError.equals(ErrorCode.OK)) 
        {
            logErrorCode(leftMasterError, "DrivetrainLeftMaster", Config.LEFT_FRONT_MOTOR, "configAllSettings");
            state = DriveBaseState.Degraded;
            System.out.println("DriveBase2020 Degraded: Line 221");
        }
        if (!rightMasterError.equals(ErrorCode.OK))
        {
            logErrorCode(rightMasterError, "DrivetrainRightMaster", Config.RIGHT_FRONT_MOTOR, "configAllSettings");
            state = DriveBaseState.Degraded;
            System.out.println("DriveBase2020 Degraded: Line 227");
        }

        // Config the encoder and check if it worked
        ErrorCode e1 = leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);
        ErrorCode e2 = rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);
        
        if (e1.value != 0 || e2.value != 0) {
            state = DriveBaseState.Degraded;
            System.out.println("DriveBase2020 Degraded: Line 236");
            logger.severe("DRIVETRAIN ENCODER NOT WORKING - DRIVETRAIN DEGRADED - ONLY DRIVER CONTROLS ACTIVE");
            logErrorCode(e1, "DrivetrainLeftMaster", Config.LEFT_FRONT_MOTOR, "configSelectedFeedbackSensor(MagEncoderRelative)");
            logErrorCode(e2, "DrivetrainRightMaster", Config.RIGHT_FRONT_MOTOR, "configSelectedFeedbackSensor(MagEncoderRelative)");
        }
        
        // Turn on voltage compensation
        leftMaster.enableVoltageCompensation(true);
        rightMaster.enableVoltageCompensation(true);

        // Set the motor inversions
        leftMaster.setInverted(Config.LEFT_FRONT_INVERTED);
        rightMaster.setInverted(Config.RIGHT_FRONT_INVERTED);
        if(leftFollower != null){
            leftFollower.setInverted(Config.LEFT_REAR_INVERTED);
        }
        if(rightFollower != null){
            rightFollower.setInverted(Config.RIGHT_REAR_INVERTED);
        }    

        // set the encoder inversions
        leftMaster.setSensorPhase(Config.DRIVETRAIN_LEFT_SENSORPHASE);
        rightMaster.setSensorPhase(Config.DRIVETRAIN_RIGHT_SENSORPHASE);

        e1 = leftMaster.setSelectedSensorPosition(0);
        e2 = rightMaster.setSelectedSensorPosition(0);

        if (e1.value != 0 || e2.value != 0) {
            state = DriveBaseState.Degraded;
            System.out.println("DriveBase2020 Degraded: Line 265");
        }

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
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        if(leftFollower != null){
            leftFollower.setNeutralMode(NeutralMode.Coast);
        }
        if(rightFollower != null){
            rightFollower.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void setBrakeMode() {
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        if(leftFollower != null){
            leftFollower.setNeutralMode(NeutralMode.Brake);
        }
        if(rightFollower != null){
            rightFollower.setNeutralMode(NeutralMode.Brake); 
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
        ErrorCode leftError = leftMaster.setSelectedSensorPosition(0, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);
        ErrorCode rightError = rightMaster.setSelectedSensorPosition(0, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);
            
        logErrorCode(leftError, "DrivetrainLeftMaster", Config.LEFT_FRONT_MOTOR, "setSelectedSensorPosition(0)");
        logErrorCode(rightError, "DrivetrainRighttMaster", Config.RIGHT_FRONT_MOTOR, "setSelectedSensorPosition(0)");

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
        leftMaster.selectProfileSlot(slotId, Config.TALON_PRIMARY_PID);
        rightMaster.selectProfileSlot(slotId, Config.TALON_PRIMARY_PID);
    }

    @Override
    public void tankDriveVelocities(double leftVel, double rightVel, double leftFF, double rightFF) {
        leftMaster.set(ControlMode.Velocity, metersPerSecondToTalonVelocity(leftVel), 
                DemandType.ArbitraryFeedForward, leftFF / 12.0);

        rightMaster.set(ControlMode.Velocity, metersPerSecondToTalonVelocity(rightVel), 
                DemandType.ArbitraryFeedForward, rightFF / 12.0); 

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
        //motion magic position close-loop control
        leftMaster.set(ControlMode.MotionMagic, metersToTalonPosistion(leftPos));
        rightMaster.set(ControlMode.MotionMagic, metersToTalonPosistion(rightPos));

        differentialDrive.feed();
    }

    /**
     * Obtain the measured raw velocities on both sides
     * @return double array
     */
    @Override
    public double[] getMeasuredVelocities() {
        double leftVel = leftMaster.getSelectedSensorVelocity();
        double rightVel = rightMaster.getSelectedSensorVelocity();
        return new double[]{leftVel, rightVel};
    }

    /**
     * Obtain the measured velocity in m/s on both sides
     * @return double array
     */
    @Override
    public double[] getMeasuredMetersPerSecond() {
        double[] velTalonUnits = getMeasuredVelocities();
        double leftVel = talonVelocityToMetersPerSecond(velTalonUnits[0]);
        double rightVel = talonVelocityToMetersPerSecond(velTalonUnits[1]);
        return new double[]{leftVel, rightVel};
    }

    /**
     * Obtain left encoder position in meters
     * @return double
     */
    @Override
    public double getLeftPosition() {
        return talonPosistionToMeters(leftMaster.getSelectedSensorPosition());
    }

    /**
     * Obtain right encoder position in meters
     * @return double
     */
    @Override
    public double getRightPosition() {
        return talonPosistionToMeters(rightMaster.getSelectedSensorPosition());
    }

    /**
     * Obtain left encoder position in raw unit
     * @return double
     */
    @Override
    public double getLeftEncoderPosition() {
        return leftMaster.getSelectedSensorPosition();
    }

    /**
     * Obtain right encoder position in raw unit
     * @return double
     */
    @Override
    public double getRightEncoderPosition() {
        return rightMaster.getSelectedSensorPosition();
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

    /**
     * Converting talon ticks/100ms to m/s
     * 
     * Unit Conversion Method
     */
    private double talonVelocityToMetersPerSecond(double talonVelocity) {
        return talonPosistionToMeters(talonVelocity * 10); // Convert ticks/100ms to ticks/sec
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

