// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.config.Config;

public class intakePneumaticSubsystem extends SubsystemBase {
  DoubleSolenoid intakeUpDown;
  DoubleSolenoid intakeFloat;
  private static final intakePneumaticSubsystem INTAKE_PNEUMATIC_SUBSYSTEM = new intakePneumaticSubsystem();
  
  /** Creates a new intakePneumaticSubsystem. */
  public intakePneumaticSubsystem() 
  {
    intakeUpDown = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                      PneumaticsModuleType.CTREPCM, 
                                      Config.INTAKE_PNEUMATIC_FORWARD_CHANNEL_1, 
                                      Config.INTAKE_PNEUMATIC_REVERSE_CHANNEL_1);

    intakeFloat = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                     PneumaticsModuleType.CTREPCM, 
                                     Config.INTAKE_PNEUMATIC_FORWARD_CHANNEL_2, 
                                     Config.INTAKE_PNEUMATIC_REVERSE_CHANNEL_2); 
  }

  public boolean isActive()
  {
    if(Config.CTRE_PCM_CAN_ID == -1 ||
       Config.INTAKE_PNEUMATIC_REVERSE_CHANNEL_1 == -1 || 
       Config.INTAKE_PNEUMATIC_FORWARD_CHANNEL_1 == -1 ||
       Config.INTAKE_PNEUMATIC_REVERSE_CHANNEL_2 == -1 ||
       Config.INTAKE_PNEUMATIC_FORWARD_CHANNEL_2 == -1)
    {
      return false;
    } 
    else
    {
      return true;
    }
  }
   /**
   * Returns the singleton instance for the ShooterSubsystem
   */
  public static intakePneumaticSubsystem getInstance()
  {
    if (INTAKE_PNEUMATIC_SUBSYSTEM.isActive())
      return INTAKE_PNEUMATIC_SUBSYSTEM;
    else
      return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveDown()
  {
    intakeUpDown.set(Value.kReverse);
    //@todo: Determine which state for float
    intakeFloat.set(Value.kReverse);
  }

  public void moveUp()
  {
    intakeFloat.set(Value.kOff);
    intakeUpDown.set(Value.kForward);
  }

  public void stop()
  {
    intakeUpDown.set(Value.kOff);
    intakeFloat.set(Value.kOff);
  }
}
