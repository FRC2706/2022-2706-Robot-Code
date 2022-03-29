// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.SwitchSubsystem;

public class IndexCargoOnce extends CommandBase {

    /**
     * CONSTANTS
     */
    private final int zeroCargoIncrementPastSwitch = 10;
    private final int rpmForIndexing = 700;


    
    private boolean reachedFirstLimitSwitch = false;
    private int numCargoAtBeginning = 0;

    /** Creates a new IndexCargoOnce. */
    public IndexCargoOnce() {
        addRequirements(IndexerSubSystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        numCargoAtBeginning = IndexerSubSystem.getInstance().numCargo;
        switch (numCargoAtBeginning) {
            case 0:
                reachedFirstLimitSwitch = false;
            
            case 1:
                IndexerSubSystem.getInstance().numCargo++;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (numCargoAtBeginning) {
            case 0:
                if (reachedFirstLimitSwitch == false) {
                    if (SwitchSubsystem.getInstance().isSwitchOnePressed()) {
                        reachedFirstLimitSwitch = true;
                        IndexerSubSystem.getInstance().numCargo++; 
                        IndexerSubSystem.getInstance().setIndexerPosition();
                    } else {
                        IndexerSubSystem.getInstance().setTargetRPM(rpmForIndexing);
                    }
                    
                }
                if (reachedFirstLimitSwitch == true) {
                    IndexerSubSystem.getInstance().runForIntake(zeroCargoIncrementPastSwitch);
                }

            case 1:
                IndexerSubSystem.getInstance().setTargetRPM(rpmForIndexing);   
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IndexerSubSystem.getInstance().stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (SwitchSubsystem.getInstance().isSwitchTwoPressed()) {
            return true;
        }
        else if (numCargoAtBeginning == 0 && IndexerSubSystem.getInstance().isAtTargetPosition()) {
            return true;
        }
        return false;
    }
}
