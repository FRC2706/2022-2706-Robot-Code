// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SwitchSubsystem extends SubsystemBase {
  private DigitalInput limitSwitch;
  private boolean currentState;

  /** Creates a new SwithSubsytem. */
public SwitchSubsystem(int switchPort) {
    // Initialize the subsystem if the shooter exists
    if (switchPort != -1) {
      limitSwitch = new DigitalInput(switchPort);
      currentState = limitSwitch.get();
    }
    else
    {
      limitSwitch = null;
    }

  }
  public boolean isActive() {
    return limitSwitch != null;
}
public boolean isStateChanged(){
    //detect the state change
    //then it doesn't matter the open state is true of false
    //will have two close detections
    if(limitSwitch.get() != currentState)
    {
      currentState = limitSwitch.get();
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
    return limitSwitch.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
