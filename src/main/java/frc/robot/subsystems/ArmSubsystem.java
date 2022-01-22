package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ReverseFeeder;
import frc.robot.config.Config;

public class ArmSubsystem extends ConditionalSubsystemBase {

    // TODO Change placeholder values to actual limits
    private static final int FORWARD_LIMIT_TICKS = 885;//Config.robotSpecific(4150, 2200);
    private static final int REVERSE_LIMIT_TICKS = 0;//Config.robotSpecific(3500, 1300);

    // Tick count of the arm at horizontal. In this case the lower limit is horizontal
    private static final int ARM_HORIZONTAL_TICKS = REVERSE_LIMIT_TICKS;

    private static final int acceptableError = 50;

    private static ArmSubsystem INSTANCE = new ArmSubsystem();

    private int currentPosition = REVERSE_LIMIT_TICKS;

    WPI_TalonSRX armTalon;
    ErrorCode errorCode;

    private static final int[] setpoints = {
            0,
            300,
            600,
            850
        };


    private ArmSubsystem() {

        // Init all the talon values
        armTalon = new WPI_TalonSRX(Config.ARM_TALON);

        // Config factory default to clear out any lingering values
        armTalon.configFactoryDefault();

        // Allow the arm to be moved easily when disabled
        // WOULD DESTORY GEARBOX IF ARMS MOVED MANUALLY WHILE IN BRAKE MODE
        armTalon.setNeutralMode(NeutralMode.Coast);

        // Setup the talon, recording the error code
        errorCode = armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
                0, Config.CAN_TIMEOUT_SHORT);

        SmartDashboard.putNumber("Arm Error Code", errorCode.value);
        System.out.println("Arm error code : " + errorCode.name());
        System.out.println("Arm Encoder: " + armTalon.getSelectedSensorPosition(0));

        armTalon.setInverted(Config.INVERT_ARM_TALON);

        // /* Config the peak and nominal outputs, 12V means full */
        // armTalon.configNominalOutputForward(0, Config.CAN_TIMEOUT_SHORT);
        // armTalon.configNominalOutputReverse(0, Config.CAN_TIMEOUT_SHORT);
        // armTalon.configPeakOutputForward(1, Config.CAN_TIMEOUT_SHORT);
        // armTalon.configPeakOutputReverse(-1, Config.CAN_TIMEOUT_SHORT);

        armTalon.configAllowableClosedloopError(0, Config.ARM_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS, Config.CAN_TIMEOUT_SHORT);

        //  Config the PID Values based on constants
        armTalon.config_kP(0, Config.ARM_PID_P, Config.CAN_TIMEOUT_LONG);
        armTalon.config_kI(0, Config.ARM_PID_I, Config.CAN_TIMEOUT_LONG);
        armTalon.config_kD(0, Config.ARM_PID_D, Config.CAN_TIMEOUT_LONG);
        armTalon.config_kF(0, Config.ARM_PID_F, Config.CAN_TIMEOUT_LONG);
        armTalon.config_IntegralZone(0, Config.ARM_PID_IZONE, Config.CAN_TIMEOUT_LONG);

        armTalon.configMotionCruiseVelocity(Config.ARM_PID_CRUISE_VELOCITY, Config.CAN_TIMEOUT_SHORT);
        armTalon.configMotionAcceleration(Config.ARM_PID_ACCELERATION, Config.CAN_TIMEOUT_SHORT);
        armTalon.configMotionSCurveStrength(Config.ARM_PID_SCURVE, Config.CAN_TIMEOUT_SHORT);
     


        // Set up the close loop period
        // armTalon.configClosedLoopPeriod(0, Config.CAN_TIMEOUT_LONG);
        armTalon.setSensorPhase(Config.ARM_PHASE);
        // armTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Config.CAN_TIMEOUT_LONG);
        // armTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Config.CAN_TIMEOUT_LONG);


        //    Enable forward soft limit and set the value in encoder ticks
        armTalon.configForwardSoftLimitEnable(true);
        armTalon.configForwardSoftLimitThreshold(FORWARD_LIMIT_TICKS, Config.CAN_TIMEOUT_LONG);
        
        // Limit switch 
        // armTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Config.CAN_TIMEOUT_SHORT);
        armTalon.configClearPositionOnLimitR(true, Config.CAN_TIMEOUT_SHORT);



        // Max voltage to apply with the talon. 12 is the maximum
        // armTalon.configVoltageCompSaturation(12, Config.CAN_TIMEOUT_LONG);
        // armTalon.enableVoltageCompensation(true);

        // Number of seconds from 0 to full throttle
     //   armTalon.configOpenloopRamp(0.6, Config.CAN_TIMEOUT_LONG);


        createCondition("talonFunctional", SubsystemConditionStates.ALWAYS);

        SubsystemCondition talonErrorCondition = getCondition("talonFunctional");

        if (errorCode.value == 0) {
            talonErrorCondition.setState(true);
        }

        if (Config.robotId != 0) {
            armTalon.setSelectedSensorPosition(0);
        }
        currentPosition = (int) armTalon.getSelectedSensorPosition();
    }

    public void addToCurrentPosition(int increment) {
        if(this.currentPosition + increment >= FORWARD_LIMIT_TICKS) {
            currentPosition = FORWARD_LIMIT_TICKS;
        } else if(this.currentPosition + increment <= REVERSE_LIMIT_TICKS) {
            currentPosition = REVERSE_LIMIT_TICKS;
        } else {
            this.currentPosition += increment;
        }
    }

    /**
     * Return the singleton arm instance. Instantiates instance if not already done so.
     *
     * @return the Arm instance
     */
    public static ArmSubsystem getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ArmSubsystem();
        
        return INSTANCE;
    }

    /**
     * Set the talon to 0 encoder ticks
     */
    public void zeroTalonEncoder() {
        armTalon.setSelectedSensorPosition(0);
    }

    public void setpoint(int setpointIndex) {
        if(setpointIndex < setpoints.length) {
            currentPosition = setpoints[setpointIndex];
        } else {
            DriverStation.reportError("Invalid arm position [Array index out of bounds]", false);
        }
    }

    public void moveArm(double speed) {
        armTalon.set(speed);
    }
    
    @Override
    public void periodic() {
        // super.periodic();

        double currentTicks = armTalon.getSelectedSensorPosition(0);

        SmartDashboard.putNumber("Lower Limit", REVERSE_LIMIT_TICKS);
        SmartDashboard.putNumber("Upper Limit", FORWARD_LIMIT_TICKS);
        // if (Config.ARM_TALON != -1)
        SmartDashboard.putNumber("Arm Motor Ticks", currentTicks);
        SmartDashboard.putNumber("Arm Angle", toDeg(currentTicks));
        SmartDashboard.putNumber("Desired Position", currentPosition);
        SmartDashboard.putNumber("Arm Pos Error", currentPosition - currentTicks);

        
        // Set the desired position every cycle and update the gravity compensation at the current angle.
        double currentAngleFromHorizontal = toDeg(currentTicks - ARM_HORIZONTAL_TICKS);
        double gravityCompensation = getArbFeedforward(currentAngleFromHorizontal);
        
        if (currentPosition < 3 && currentTicks < 3) {
            armTalon.set(-0.1);
        } else {
            armTalon.set(ControlMode.MotionMagic, currentPosition, DemandType.ArbitraryFeedForward, gravityCompensation);
        }

        

        SmartDashboard.putNumber("Arm Gravity Compensation", gravityCompensation);
        SmartDashboard.putNumber("Arm Error", armTalon.getClosedLoopError()); 
        SmartDashboard.putBoolean("Arm Limit Switch", armTalon.getSensorCollection().isRevLimitSwitchClosed());
    
    }

    public boolean reachedSetpoint(int index) {
        return reachedPosition(setpoints[index]);
    }

    public boolean reachedPosition(int position) {
        return Math.abs(armTalon.getSelectedSensorPosition() - position) < acceptableError; 
    }

    public void setPosition(int ticks) {
        currentPosition = ticks;
    }

    /**
     * @param units CTRE mag encoder sensor units
     * @return degrees rounded to tenths.
     */
    Double toDeg(double units) {
        double deg = units * 360.0 / 4096.0;

        /* truncate to 0.1 res */
        deg *= 10;
        deg = (int) deg;
        deg /= 10;

        return deg;
    }

    /**
     * Feedforward for Gravity and Spring Compensation
     * 
     * @param angle Angle considering 0 degrees as horizontal. 
     *          (arm has to move up slightly to get to horiztonal,
     *           so 0 encoder ticks doesn't mean horizontal)
     * 
     * @return ArbFF value between -1 & 1 to pass as a DemandType.ArbitraryFeedforward to talon
     */
    private double getArbFeedforward(double deg) {

        // Equation that takes into account spring and gravity.
        // Gravity of an arm can be mapped with a sin or cos function.
        // The spring equation is a mechanical thing that D&F gave me.
        // Equation is V=5.51×sin(∅+10°)-0.062×(∅-5°)
        // V for volts and ∅ for angle (math was simplier if 0 degrees as arm is vertical)
        // Note this equation is specfic to the torsion spring on the 2020 robot 
        // I recommand asking D&F for an equation to match an arm with a spring in future years.

        // java.lang.Math.sin needs radians

        // Equation demands a the complementary angle
        double theta = Math.toRadians(90 - (deg - 10));

        SmartDashboard.putNumber("ArmGComp-Angle", Math.toDegrees(theta));

        double springTorsionRate = 0.062*10; // Should be accurate, may need to be updated
        double torqueAtHorizontal = 3.0;// 5.51; // UPDATE BASED ON MEASURED VALUES
        double springPreloadedAngle = Math.toRadians(-7); // Should be accurate, may need to be updated
        // Spring doesn't generate torque until it travels 10 degrees so its negatively preloaded.

        double volts = torqueAtHorizontal * Math.sin(theta) - springTorsionRate * (theta - springPreloadedAngle); // 5.51 and 5 may need to be adjusted
        
        double arbFF = volts / armTalon.getBusVoltage();

        return arbFF;
    }

    /** 
     * Getters and setters, maybe temporay, to make lower arm work
    */
    public int getPosistion() {
        return (int) armTalon.getSelectedSensorPosition();
    }
    public void setPosition(int ticks, double arbFF) {
        armTalon.set(ControlMode.Position, ticks, DemandType.ArbitraryFeedForward, arbFF);
    }
    public void resetPosition(int ticks) {
        armTalon.setSelectedSensorPosition(ticks);
    }
    public void stopMotor() {
        armTalon.stopMotor();
    }



}
