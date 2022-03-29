// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.IntakeCargoSubsystem;

public class IntakeAndIndexer extends CommandBase {
    /**
     * CONSTANTS
     */
    private final int INDEXER_RPM = 1000;

    private IndexerSubSystem indexerSubsystem;
    private IntakeCargoSubsystem intakeSubsystem;
    
    private RecoverIndexer recoverIndexCommand;

    /** Creates a new IntakeAndIndexer. */
    public IntakeAndIndexer() {
        indexerSubsystem = IndexerSubSystem.getInstance();
        intakeSubsystem = IntakeCargoSubsystem.getInstance();
        
        recoverIndexCommand = new RecoverIndexer();

        addRequirements(indexerSubsystem, intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.start();
        indexerSubsystem.setTargetRPM(INDEXER_RPM);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        indexerSubsystem.stop();
        recoverIndexCommand.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
