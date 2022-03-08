// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SwitchSubsystem extends SubsystemBase {
  private DigitalInput limitSwitch;

  /** Creates a new SwithSubsytem. */
  public SwitchSubsystem(int switchPort) {
    // Initialize the subsystem if the shooter exists
    if (switchPort != -1) {
      limitSwitch = new DigitalInput(switchPort);
    }
    else
    {
      limitSwitch = null;
    }

  }
  public boolean isActive() {
    return limitSwitch != null;
}
  public boolean isDetected(){
    //switch default state is open
    return  limitSwitch.get();
    //to double check the value of get
  }

  public boolean getResult()
  {
    return limitSwitch.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
