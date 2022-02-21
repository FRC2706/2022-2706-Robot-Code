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

    //This command with input RPM and timeout. For debugging.
    public SpinUpShooterWithTime(int RPM, int timeout) {
        targetRPM = RPM;
        this.timeout   = timeout;

        shooterSubsystem = ShooterSubsystem.getInstance();
        if (shooterSubsystem != null) 
        {
            addRequirements(shooterSubsystem);
        }

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
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
        //print debug info here
        SmartDashboard.putNumber("shooter measured RPM", shooterSubsystem.getMeasuredRPM());
        SmartDashboard.putNumber("shooter target RPM", targetRPM);
        SmartDashboard.putNumber("shooter error RPM", targetRPM-shooterSubsystem.getMeasuredRPM());
        SmartDashboard.putNumber("shooter temp", shooterSubsystem.getTemperature());
        SmartDashboard.putNumber("shooter current", shooterSubsystem.getCurrentDraw());
        SmartDashboard.putBoolean("shooter isTargetRPM", shooterSubsystem.isAtTargetRPM());
        
        //stop the shooter
        shooterSubsystem.setTargetRPM(0);       
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > timeout;
    }

}
