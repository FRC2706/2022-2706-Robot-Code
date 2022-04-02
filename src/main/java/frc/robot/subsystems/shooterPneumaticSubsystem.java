// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class shooterPneumaticSubsystem extends SubsystemBase {
  DoubleSolenoid doubleSolenoidShooter;
  DoubleSolenoid doubleSolenoidFloat;
  private static final shooterPneumaticSubsystem SHOOTER_PNEUMATIC_SUBSYSTEM = new shooterPneumaticSubsystem();

  /** Creates a new shooterPneumaticSubsystem. */
  public shooterPneumaticSubsystem() 
  {
    
    if (Config.CTRE_PCM_CAN_ID == -1 
    || Config.KICKER_PNEUMATIC_FORWARD_CHANNEL == -1 
    || Config.KICKER_PNEUMATIC_REVERSE_CHANNEL == -1)
    {
      doubleSolenoidShooter = null;
    }
    else
    {
    doubleSolenoidShooter = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, 
                                               PneumaticsModuleType.CTREPCM,
                                               Config.KICKER_PNEUMATIC_FORWARD_CHANNEL,
                                               Config.KICKER_PNEUMATIC_REVERSE_CHANNEL);
    }

    if(Config.CTRE_PCM_CAN_ID == -1 
    || Config.KICKER_PNEUMATIC_FLOAT_CHANNEL_1 == -1 
    || Config.KICKER_PNEUMATIC_FLOAT_CHANNEL_2 == -1)
    {
      doubleSolenoidFloat = null;
    }
    else
    {
      doubleSolenoidFloat = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, 
                                              PneumaticsModuleType.CTREPCM,
                                              Config.KICKER_PNEUMATIC_FLOAT_CHANNEL_1,
                                              Config.KICKER_PNEUMATIC_FLOAT_CHANNEL_2);
    }
  }

  public boolean isActive()
  {
    if(doubleSolenoidFloat == null || doubleSolenoidShooter == null)
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
  public static shooterPneumaticSubsystem getInstance()
  {
    if (SHOOTER_PNEUMATIC_SUBSYSTEM.isActive())
      return SHOOTER_PNEUMATIC_SUBSYSTEM;
    else
      return null;
  }

  public void moveDown()
  {
    doubleSolenoidFloat.set(Value.kForward);
    doubleSolenoidShooter.set(Value.kReverse);

  }

  public void moveUp()
  {
    doubleSolenoidFloat.set(Value.kForward);
    doubleSolenoidShooter.set(Value.kForward);
  }

  public void setToFloatForward()
  {
    doubleSolenoidFloat.set(Value.kForward);
  }

  public void setFloatMode()
  {
    doubleSolenoidFloat.set(Value.kReverse);
    doubleSolenoidShooter.set(Value.kForward);
  }

  public void stopShooterPneumatic()
  {
    doubleSolenoidShooter.set(Value.kOff);
    doubleSolenoidFloat.set(Value.kOff);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
