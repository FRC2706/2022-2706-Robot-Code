// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
public class SwitchSubsystem extends SubsystemBase {
  private DigitalInput limitSwitchOne;
  private DigitalInput limitSwitchTwo;
  private boolean currentState;

  private static SwitchSubsystem instance = null;

  public static SwitchSubsystem getInstance() {
    if (instance == null) {
      instance = new SwitchSubsystem();
    }
    return instance;
  }


  /** Creates a new SwithSubsytem. */
  private SwitchSubsystem() {
  
    // Initialize the subsystem if the shooter exists
    if (Config.INDEXER_SWITCH_ONE != -1) {
      limitSwitchOne = new DigitalInput(Config.INDEXER_SWITCH_ONE);
      currentState = limitSwitchOne.get();
    }
    else
    {
      limitSwitchOne = null;
    }

    if (Config.INDEXER_SWITCH_TWO != -1) {
      limitSwitchTwo = new DigitalInput(Config.INDEXER_SWITCH_TWO);
    }

  }
  public boolean isActive() {
    return limitSwitchOne != null;
}
public boolean isStateChanged(){
    //detect the state change
    //then it doesn't matter the open state is true of false
    //will have two close detections
    if(limitSwitchOne.get() != currentState)
    {
      currentState = limitSwitchOne.get();
      return true;
    }
    else
    {
      return false;
    }
  }

  //Open is false, closed is true
  public boolean getResult()
  {
    //@todo: add a counter for detection
    //
    return limitSwitchOne.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isSwitchOnePressed() {
    return !limitSwitchOne.get();
  } 

  public boolean isSwitchTwoPressed() {
    return !limitSwitchTwo.get();
  } 



}
