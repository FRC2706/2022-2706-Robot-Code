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
  private static final shooterPneumaticSubsystem SHOOTER_PNEUMATIC_SUBSYSTEM = new shooterPneumaticSubsystem();

  /** Creates a new shooterPneumaticSubsystem. */
  public shooterPneumaticSubsystem() 
  {
    //@todo: Check the channel ports
    doubleSolenoidShooter = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, 
                                               PneumaticsModuleType.CTREPCM,
                                               Config.SHOOTER_PNEUMATIC_FORWARD_CHANNEL,
                                               Config.SHOOTER_PNEUMATIC_REVERSE_CHANNEL);
  }

  public boolean isActive()
  {
    if(Config.CTRE_PCM_CAN_ID == -1 || 
       Config.SHOOTER_PNEUMATIC_REVERSE_CHANNEL == -1 || 
       Config.SHOOTER_PNEUMATIC_FORWARD_CHANNEL == -1)
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

  public void moveUp()
  {
    doubleSolenoidShooter.set(Value.kForward);
  }

  public void moveDown()
  {
    doubleSolenoidShooter.set(Value.kReverse);
  }

  public void stop()
  {
    doubleSolenoidShooter.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
