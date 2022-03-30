// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwitchSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class RecoverIndexer extends CommandBase {
    /**
     * CONSTANTS
     */
    private final int INDEXER_RPM = -500;
    private final int SHOOTER_RPM = -1000;
    private final double TIMEOUT = 1;


    private IndexerSubSystem indexerSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private SwitchSubsystem switchSubsystem;

    private Timer timer;

    /** Creates a new RecoverIndexer. */
    public RecoverIndexer() {
        indexerSubsystem = IndexerSubSystem.getInstance();
        shooterSubsystem = ShooterSubsystem.getInstance();
        switchSubsystem = SwitchSubsystem.getInstance();

        timer = new Timer();
        addRequirements(indexerSubsystem, shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexerSubsystem.setTargetRPM(INDEXER_RPM);
        shooterSubsystem.setTargetRPM(SHOOTER_RPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        indexerSubsystem.stop();
        shooterSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return switchSubsystem.isSwitchTwoPressed() || switchSubsystem.isSwitchOnePressed() || timer.hasElapsed(TIMEOUT);
    }
}
