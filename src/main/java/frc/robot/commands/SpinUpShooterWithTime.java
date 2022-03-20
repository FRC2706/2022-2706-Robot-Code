package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooterWithTime extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private int targetRPM;
    private double timeout;
    
    private Timer timer;
    private boolean m_bUseTimer;

    //Only when inputTimeout > 0, we will use the timer.
    public SpinUpShooterWithTime(int RPM, int inputTimeout) {
        targetRPM = RPM;
        timeout   = inputTimeout;

        if ( timeout > 0 )
        {
            m_bUseTimer = true;
            timer = new Timer();
        }
        else
        {
            m_bUseTimer = false;
        }

        shooterSubsystem = ShooterSubsystem.getInstance();
        if (shooterSubsystem != null) 
        {
            addRequirements(shooterSubsystem);
        }
    }

    @Override
    public void initialize() {
        if( m_bUseTimer == true )
        { 
            timer.start();
            timer.reset();
        }
    }

    @Override
    public void execute() {
        if( shooterSubsystem != null )
        {
            shooterSubsystem.setTargetRPM(targetRPM);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if( shooterSubsystem != null )
        {
            //print debug info here
            SmartDashboard.putNumber("shooter measured RPM", shooterSubsystem.getMeasuredRPM());
            SmartDashboard.putNumber("shooter target RPM", targetRPM);
            SmartDashboard.putNumber("shooter error RPM", targetRPM-shooterSubsystem.getMeasuredRPM());
            SmartDashboard.putNumber("shooter temp", shooterSubsystem.getTemperature());
            SmartDashboard.putNumber("shooter current", shooterSubsystem.getCurrentDraw());
            SmartDashboard.putBoolean("shooter isTargetRPM", shooterSubsystem.isAtTargetRPM());
            
            //stop the shooter
            shooterSubsystem.stop();
        }

        if( m_bUseTimer == true )
          timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if( m_bUseTimer == true )
            return timer.get() > timeout;
        else
            return false;
    }

}
