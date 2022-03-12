package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.config.Config;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private WPI_TalonSRX m_climber;

    private ClimberSubsystem() {

        // Initialize the subsystem if the shooter exists
        if (Config.CLIMBER_TALON != -1) {
            initializeSubsystem();
        }
        else{
            m_climber = null;
        }
    }

    /**
     * Initialization process for the shooter to be run on robots with this
     * mechanism.
     */
    private void initializeSubsystem() {
        m_climber = new WPI_TalonSRX(Config.CLIMBER_TALON);

        m_climber.setInverted(true);
    }


    /**
     * Run the climb motor ensuring that it is positive
     */
    public void climb() {
        m_climber.set(1.0);
        m_climber.set(ControlMode.PercentOutput, 0.05);
    
    }
    /**
     * Stops the motor
     */
    public void stopClimberMotor() {
        m_climber.set(0);
    }



    public boolean isActive() {
        return m_climber != null;
    }

    private static class ClimberHolder {
        private static final ClimberSubsystem INSTANCE_CLIMBER = new ClimberSubsystem();
    }

    /**
     * Returns the singleton instance for the ShooterSubsystem
     */
    public static ClimberSubsystem getInstance() {
        if(ClimberHolder.INSTANCE_CLIMBER.isActive()==true){
            return ClimberHolder.INSTANCE_CLIMBER;
        }
        else{
            return null;
        }
    }

    /**
     * Return the motor temperature (Celsius) as measured by the encoder
     */
    public double getTemperature() {
        return m_climber.getTemperature();
    }

    /**
     * Return the motor current draw measured by the encoder
     */
    public double getSupplyCurrent() {
        return m_climber.getSupplyCurrent();
    }

    public double getMotorOutputPercent() {
        return m_climber.getMotorOutputPercent();
    }

    @Override
    public void periodic() {
        
    }
}