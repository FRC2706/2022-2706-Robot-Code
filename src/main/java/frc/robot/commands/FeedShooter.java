// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwitchSubsystem;

public class FeedShooter extends CommandBase {

    private static final FluidConstant<Integer> feedingSpeedRPM = new FluidConstant<>("Feeding RPM", 700)
    .registerToTable(Config.constantsTable);

    private int feedingState = 0;

    /** Creates a new feedShooter. */
    public FeedShooter() {
        addRequirements(IndexerSubSystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        switch (IndexerSubSystem.getInstance().numCargo) {
            case 2:
                feedingState = 6;
            case 1: 
                feedingState = 3;
            case 0:
                this.cancel();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (feedingState) {
            /** Start here for two cargo  **/
            // Feed until cargo is aligned at the second switch
            case 6:
                if (SwitchSubsystem.getInstance().isSwitchTwoPressed()) {
                    feedingState--;
                } else {
                    IndexerSubSystem.getInstance().setTargetRPM(feedingSpeedRPM.get());
                }

            // Wait for shooter targetRPM
            case 5:
                if (ShooterSubsystem.getInstance().isAtTargetRPM()) {
                    feedingState--;
                }
            
            // Feed cargo into shooter until second switch is unpressed
            case 4: 
                if (SwitchSubsystem.getInstance().isSwitchTwoPressed()) {
                    IndexerSubSystem.getInstance().setTargetRPM(feedingSpeedRPM.get());
                } else {
                    IndexerSubSystem.getInstance().stop();
                    IndexerSubSystem.getInstance().numCargo = 1;
                    feedingState--;
                }

            /** Start here for one cargo  **/
            // Move cargo until it hits the second switch
            case 3:
                if (SwitchSubsystem.getInstance().isSwitchTwoPressed()) {
                    feedingState--;
                } else {
                    IndexerSubSystem.getInstance().setTargetRPM(feedingSpeedRPM.get());
                }

            
            // Wait for shooter targetRPM
            case 2:
                if (ShooterSubsystem.getInstance().isAtTargetRPM()) {
                    feedingState--;
                }
            
            // Feed cargo until switch is unpressed, then stop this command
            case 1:
                if (SwitchSubsystem.getInstance().isSwitchTwoPressed()) {
                    IndexerSubSystem.getInstance().setTargetRPM(feedingSpeedRPM.get());
                } else {
                    IndexerSubSystem.getInstance().stop();
                    IndexerSubSystem.getInstance().numCargo = 0;
                    this.cancel();
                }
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
        return IndexerSubSystem.getInstance().numCargo == 0;
    }
}
