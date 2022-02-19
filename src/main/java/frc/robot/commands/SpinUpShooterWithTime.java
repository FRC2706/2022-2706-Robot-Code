package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooterWithTime extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private int targetRPM;
    private int timeout;

    //@todo
    private final int RPM_TOLERANCE = 75;

    private Timer timer;

    //This command with input RPM and time. For debugging.
    //make a new one AutomaticShooterWithTime cmd.
    public SpinUpShooterWithTime(int RPM, int time) {
        targetRPM = RPM;
        timeout   = time;

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
        SmartDashboard.putNumber("shooter measured RPM", shooterSubsystem.getRPM());
        SmartDashboard.putNumber("shooter target RPM", targetRPM);
        SmartDashboard.putNumber("shooter error RPM", targetRPM-shooterSubsystem.getRPM());
        SmartDashboard.putNumber("shooter temp", shooterSubsystem.getTemperature());
        SmartDashboard.putNumber("shooter current", shooterSubsystem.getCurrentDraw());
        SmartDashboard.putBoolean("shooter isTargetRPM", isAtTargetRPM());
        
        //stop the shooter
        shooterSubsystem.setTargetRPM(0);       
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > timeout;
    }

    /**
     * Check the actual RPM and compare it with targetRPM to verify that the shooter
     * is up to necessary speed to fire.
     */
    public boolean isAtTargetRPM() {
        double errorRPM = targetRPM-shooterSubsystem.getRPM();
        return (Math.abs(errorRPM) < RPM_TOLERANCE);
    }
}
