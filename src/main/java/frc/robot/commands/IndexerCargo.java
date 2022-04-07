// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.SwitchSubsystem;
import frc.robot.subsystems.Bling;


public class IndexerCargo extends CommandBase {

  private IndexerSubSystem indexer;
  private ColorSensorSubsystem colorSensor;
  private SwitchSubsystem switchDetector;
  private Bling bling;
  public boolean switchDetected;
  public boolean colorSensorDetected;
  public boolean colorSensorFirstDetected = false;
  public int counter = 0;
  public int nColorSensorDetectedCount = 0;
  public int numCargo = 0;

  /** Creates a new IndexerCargo. */
  public IndexerCargo() {

    indexer = IndexerSubSystem.getInstance();
    colorSensor = ColorSensorSubsystem.getInstance();
    switchDetector = new SwitchSubsystem(Config.INDEXER_SWITCH_END);
    bling = Bling.getINSTANCE();

    // Use addRequirements() here to declare subsystem dependencies.
    if ( indexer != null )
    {
      addRequirements(indexer);
    }

    if(colorSensor != null)
    {
      addRequirements(colorSensor);
    }

    if(switchDetector != null)
    {
      addRequirements(switchDetector);
    }

    if(bling != null)
    {
      addRequirements(bling);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if ( indexer != null )
      indexer.setIndexerPosition();

    nColorSensorDetectedCount = 0;
    
    counter = 0;
    numCargo = 0;
    
    switchDetected = false;
    colorSensorDetected = false;
    colorSensorFirstDetected = false;
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer == null)
      return;


    // System.out.println("auto mode: "+ Robot.m_bAutoMode);
    //System.out.println("indexer sensor: "+ indexer.m_bForIntakeGoodSensors);
   //@todo: only consider one detection, ignore all of the following detections.

   switchDetected = switchDetector.getResult();
   if (switchDetected == true) 
   {
     bling.setStrobe();
    //  System.out.println("switch detected: "+switchDetected);
     indexer.stop();
   } 
   else 
   {
     colorSensorDetected = colorSensor.isDetected();
    //  if ( colorSensorDetected == true)
    //  {
    //    System.out.println("detected one cargo");
    //  }

     if (colorSensorDetected == true && colorSensorFirstDetected == false) 
     {
       colorSensorFirstDetected = true;
       System.out.println("start shuffling the cargo "+numCargo);
     }
     
     if (colorSensorFirstDetected == true) 
     {
       bling.setRainbow();
       if(numCargo == 0)
       {
         indexer.runForIntake(11);
       }
       else 
       {
         indexer.runForIntake(4);
       }
       counter++;
       
        // TODO tune for 100
       if (counter > 100) 
       {
          // At this time, the cargo should be in the indexer, and the indexer should stop

          //reset for the next cargo 
          colorSensorFirstDetected = false;
          colorSensorDetected = false;
          counter = 0;
          indexer.setIndexerPosition();
          indexer.stop();
          numCargo++;
          System.out.println("stop shuffling the cargo "+numCargo);
        }
      } 
      else 
      {
        indexer.stop();
        //numCargo = 0;
      }
    }
   
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if ( indexer != null )
      indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
