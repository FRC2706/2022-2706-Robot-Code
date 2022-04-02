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
    if ( Config.CTRE_PCM_CAN_ID == -1 ||
        Config.INTAKE_PNEUMATIC_FORWARD_CHANNEL == -1 || 
        Config.INTAKE_PNEUMATIC_REVERSE_CHANNEL == -1 )
    {
        intakeUpDown = null;
    }
    else
      intakeUpDown = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                      PneumaticsModuleType.CTREPCM, 
                                      Config.INTAKE_PNEUMATIC_FORWARD_CHANNEL, 
                                      Config.INTAKE_PNEUMATIC_REVERSE_CHANNEL);

    if ( Config.CTRE_PCM_CAN_ID == -1 ||
         Config.INTAKE_PNEUMATIC_FLOAT_CHANNEL_1 == -1 || 
         Config.INTAKE_PNEUMATIC_FLOAT_CHANNEL_2 == -1 )
    {
        intakeFloat = null;
    }
    else
        intakeFloat = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                     PneumaticsModuleType.CTREPCM, 
                                     Config.INTAKE_PNEUMATIC_FLOAT_CHANNEL_1, 
                                     Config.INTAKE_PNEUMATIC_FLOAT_CHANNEL_2); 
  }

  public boolean isActive()
  {
    if(intakeFloat == null || intakeUpDown == null )
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
    intakeFloat.set(Value.kForward);
    intakeUpDown.set(Value.kForward); 
  }

  public void moveUp()
  {    
    intakeFloat.set(Value.kForward);
    intakeUpDown.set(Value.kReverse);
  }

  //Does not work
  public void setFloat()
  {
    intakeFloat.set(Value.kReverse);
    intakeUpDown.set(Value.kForward);  }

  public void stopFloat()
  {
   intakeFloat.set(Value.kOff);
  }

  public void stopUpDown()
  {
    intakeUpDown.set(Value.kOff);
  }

  public void stopIntakePneumatic()
  {
    intakeFloat.set(Value.kOff);
    intakeUpDown.set(Value.kOff);

  }
}
