// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class LowerArm extends CommandBase {

    private int state = 0;

    private final int ticksAtStartingPosisiton = 565;
    private final int ticksAboveStartingPosition = 750;

    private ArmSubsystem armSub;

    private int loopCount = 0;

    /** Creates a new LowerArm. */
    public LowerArm() {
        armSub = ArmSubsystem.getInstance();
        addRequirements(armSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // int currentPosistion = armSub.getPosistion();
        // int errorAllowed = 200;
        // if (currentPosistion > ticksAtStartingPosisiton+errorAllowed || currentPosistion < ticksAtStartingPosisiton-errorAllowed) {
        //     Robot.haltRobot("ARM NOT IN STARTING POSITION");
        //     this.cancel();
        //     return;
        // }
        armSub.resetPosition(ticksAtStartingPosisiton);
        armSub.moveArm(1.0);
        state = 1;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (state == 1) {
                if (armSub.reachedPosition(ticksAboveStartingPosition)) {
                    armSub.stopMotor();
                    state = 2;
                    // armSub.setPosition(0, 0.4); // 0.25
                    loopCount = 100;
                }
                else{
                    armSub.moveArm(0.7);
                }
            
            
        } else if (state == 2) {
            if (loopCount <= 0) {
                armSub.moveArm(-0.2);
                // loopCount = 100;
                state = 3;
            } else {
                loopCount--;
            }
            
        // } else if (state == 3) {
        //     if (loopCount <= 0) {
        //         armSub.stopMotor();
        //         state = 4;
        //     } else {
        //         loopCount--;
        //     }
            
        }
        


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("interrupted : " + interrupted + " state " + state);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state >= 3;
    }
}
