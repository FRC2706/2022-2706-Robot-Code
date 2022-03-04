// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AnalogSensor extends SubsystemBase {
  AnalogInput analogInput;
  /** Creates a new AanlogSensor. */
  public AnalogSensor( int analogPort) {

    if (analogPort != -1)
    {
      analogInput = new AnalogInput(analogPort);
    }
    else
      analogInput = null;
  }

  public double getVoltage()
  {
    return analogInput.getVoltage();
  }

  public double getAverageVoltage()
  {
    return analogInput.getAverageVoltage();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
