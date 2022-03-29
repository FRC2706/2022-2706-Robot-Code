// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IndexerSubSystem;

public class DetectCargoAtIndexer extends CommandBase {
    /**
     * Constants
     */
    private final double waitTimeBeforeIndex = 0;


    Command m_indexCommand; 

    /** Creates a new DetectCargoAtIndexer. */
    public DetectCargoAtIndexer() {
        addRequirements(IndexerSubSystem.getInstance(), ColorSensorSubsystem.getInstance());
        
        // m_indexCommand = new WaitCommand(waitTimeBeforeIndex).andThen(new IndexCargoOnce());
        m_indexCommand = new IndexCargoOnce();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ColorSensorSubsystem.getInstance().isDetected()) {
            if (IndexerSubSystem.getInstance().numCargo <= 2) {
                m_indexCommand.schedule();
                this.cancel();
            } else {
                // Inform driver of 3 cargo?
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
