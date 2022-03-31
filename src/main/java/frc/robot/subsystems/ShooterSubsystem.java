package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooter;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    boolean m_bGoodSensors;

    //@todo: final tuning
    private final int RPM_TOLERANCE = 75;

    //@todo: put these values as constant configs for competition
    //       or make them configurable from the network table (network listener for debug)
    // tuned PIDF values for the competition robot (Rapid React)
    public static FluidConstant<Double> P_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("P_ShooterSubsystem", 3.5e-4).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> I_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("I_ShooterSubsystem", 3.7e-7).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> D_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("D_ShooterSubsystem", 0.013).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> F_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("F_ShooterSubsystem", 0.000175).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> IZONE_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("IZONE_ShooterSubsystem", 150.0).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> TARGET_RPM = new FluidConstant<>
            ("TARGET_RPM_ShooterSubsystem", 2000.0).registerToTable(Config.constantsTable);
    //note: during the test to find a good RPM, use TARGET_RPM in setTargetRPM(int inputRPM).
   
    int targetRPM = 0;

    double kMaxOutput = 1;
    double kMinOutput = -1;

    private static final ShooterSubsystem INSTANCE_SHOOTER = new ShooterSubsystem();

    private ShooterSubsystem() {

        // Initialize the subsystem if the shooter exists
        if (Config.SHOOTER_MOTOR != -1) {
            initializeSubsystem();
        }
        else
        {
            m_shooter = null;
        }
    }

    /**
     * Initialization process for the shooter to be run on robots with this
     * mechanism.
     */
    private void initializeSubsystem() {

        REVLibError errorCode;
        m_bGoodSensors = true;
        
        m_shooter = new CANSparkMax(Config.SHOOTER_MOTOR, MotorType.kBrushless);

        // Factory Default to prevent unexpected behaviour
        m_shooter.restoreFactoryDefaults();
        m_shooter.setInverted(false);

        // PID controller for the shooter
        m_pidController = m_shooter.getPIDController();
        if ( m_pidController == null)
            m_bGoodSensors = false;

        m_encoder = m_shooter.getEncoder();
        if ( m_encoder == null )
            m_bGoodSensors = false;

        if ( m_bGoodSensors == true )
        {
            errorCode = m_pidController.setOutputRange(kMinOutput, kMaxOutput);
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setFF(F_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setP(P_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setI(I_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setD(D_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setIZone(IZONE_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);
        }

        m_shooter.setSmartCurrentLimit(80);

    }

    public boolean isActive() {
        return m_shooter != null;
    }

    /**
     * Returns the singleton instance for the ShooterSubsystem
     */
    public static ShooterSubsystem getInstance() {
        if (INSTANCE_SHOOTER.isActive())
            return INSTANCE_SHOOTER;
        else
            return null;
    }

    /**
     * Set the target RPM to ramp up to.
     */
    public void setTargetRPM(int inputRPM) {
        targetRPM = inputRPM;
        //using network table, only for testing
        //targetRPM = (int) ((double) TARGET_RPM.getValue());

        if( m_bGoodSensors == true )
        {
            m_pidController.setReference(targetRPM, ControlType.kVelocity);
        }
        else
        {
            //don't use PIDF any more.
            //@todo: test this for a fixed spot on the field.
            m_shooter.set(0.3);
        }
    }

    /**
     * Return the motor velocity (RPM) measured by the encoder
     */
    public double getMeasuredRPM() {
        if ( m_bGoodSensors == true )
            return m_encoder.getVelocity();
        else
            return 0.0;
    }

    /**
     * Return the motor temperature (Celsius) as measured by the encoder
     */
    public double getTemperature() {
        return m_shooter.getMotorTemperature();
    }

    /**
     * Return the motor current draw measured by the encoder
     */
    public double getCurrentDraw() {
        return m_shooter.getOutputCurrent();
    }

    @Override
    public void periodic() {
        
    }

    public void stop()
    {
        m_shooter.stopMotor();
    }

    /**
     * Check the actual RPM and compare it with targetRPM to verify that the shooter
     * is up to necessary speed to fire.
     */
    public boolean isAtTargetRPM() {
        double errorRPM = targetRPM - getMeasuredRPM();
        return (Math.abs(errorRPM) < RPM_TOLERANCE);
    }

    public void setPIDValues()
    {
        REVLibError errorCode = m_pidController.setFF(F_SHOOTERSUBSYSTEM.get());
        m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

        errorCode = m_pidController.setP(P_SHOOTERSUBSYSTEM.get());
        m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setI(I_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setD(D_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

            errorCode = m_pidController.setIZone(IZONE_SHOOTERSUBSYSTEM.get());
            m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);
    }
}