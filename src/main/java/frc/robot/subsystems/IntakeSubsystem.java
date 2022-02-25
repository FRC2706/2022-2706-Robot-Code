package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this IntakeSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    // The supplier of the intake speed
    //@todo: final tuning
    private final static FluidConstant<Double> INTAKE_SPEED = new FluidConstant<>("intake-target-speed", 0.5d)
            .registerToTable(Config.constantsTable);

    // The intake motor (if any)
    private CANSparkMax intakeMotor;
 
    /**
     * Creates a new instance of this IntakeSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private IntakeSubsystem() 
    {
        if (Config.INTAKE_MOTOR != -1) 
        {         
            initializeSubsystem();
        }
        else
        {
            intakeMotor = null;
        }
        
    }

    /**
     * Initialization process for the shooter to be run on robots with this
     * mechanism.
     */
    private void initializeSubsystem() {

        intakeMotor = new CANSparkMax(Config.INTAKE_MOTOR, MotorType.kBrushless);

        // Factory Default to prevent unexpected behaviour
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(60);
    }

    public boolean isActive() {
        return intakeMotor != null;
    }

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method
     * should be used -- {@code IntakeSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static IntakeSubsystem getInstance() {
        if (INSTANCE.isActive() == true)
            return INSTANCE;
        else
            return null;
    }

    public void runIntake()
    {
        intakeMotor.set(INTAKE_SPEED.get());
    }

    public void stopIntake()
    {
        intakeMotor.set(0d);
    }

    @Override
    public void periodic() {

    }
}
