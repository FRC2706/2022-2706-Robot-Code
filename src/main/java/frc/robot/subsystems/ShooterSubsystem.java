package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooter;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    //@todo: final tuning
    private final int RPM_TOLERANCE = 75;

    //@todo: put these values as constant configs for competition
    //       or make them configurable from the network table (network listener for debug)
    // PID values (currently set for protobot's shooter)
    public static FluidConstant<Double> P_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("P_ShooterSubsystem", 0.0025).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> I_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("I_ShooterSubsystem", 0.0).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> D_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("D_ShooterSubsystem", 0.004).registerToTable(Config.constantsTable);

    public static FluidConstant<Double> F_SHOOTERSUBSYSTEM = new FluidConstant<>
            ("F_ShooterSubsystem", 0.0002).registerToTable(Config.constantsTable);

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
        m_shooter = new CANSparkMax(Config.SHOOTER_MOTOR, MotorType.kBrushless);

        // Factory Default to prevent unexpected behaviour
        m_shooter.restoreFactoryDefaults();

        // PID controller for the shooter
        m_pidController = m_shooter.getPIDController();
        m_encoder = m_shooter.getEncoder();

        m_shooter.setInverted(true);

        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setFF(F_SHOOTERSUBSYSTEM.get());
        m_pidController.setP(P_SHOOTERSUBSYSTEM.get());
        m_pidController.setI(I_SHOOTERSUBSYSTEM.get());
        m_pidController.setD(D_SHOOTERSUBSYSTEM.get());

        m_shooter.setSmartCurrentLimit(60);

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
        m_pidController.setReference(targetRPM, ControlType.kVelocity);

    }

    /**
     * Return the motor velocity (RPM) measured by the encoder
     */
    public double getMeasuredRPM() {
        return m_encoder.getVelocity();
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

    /**
     * Check the actual RPM and compare it with targetRPM to verify that the shooter
     * is up to necessary speed to fire.
     */
    public boolean isAtTargetRPM() {
        double errorRPM = targetRPM - getMeasuredRPM();
        return (Math.abs(errorRPM) < RPM_TOLERANCE);
    }
}