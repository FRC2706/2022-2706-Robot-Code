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

public class IndexerCargo extends CommandBase {

  private IndexerSubSystem indexer;
  private ColorSensorSubsystem colorSensor;
  public boolean switchDetected;
  public boolean colorSensorDetected;
  public boolean colorSensorFirstDetected = false;
  public int counter = 0;
  public int nColorSensorDetectedCount = 0;
  public boolean bFirstSwitchDetected;

  /** Creates a new IndexerCargo. */
  public IndexerCargo() {

    indexer = IndexerSubSystem.getInstance();
    colorSensor = ColorSensorSubsystem.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    if ( indexer != null )
    {
      addRequirements(indexer);
    }

    if(colorSensor != null)
    {
      addRequirements(colorSensor);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if ( indexer != null )
      indexer.setIndexerPosition();

    nColorSensorDetectedCount = 0;
    colorSensorFirstDetected = false;
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer == null)
      return;

      //System.out.println("auto mode: "+ Robot.m_bAutoMode);
      // System.out.println("indexer sensor: "+ indexer.m_bForIntakeGoodSensors);

    if (colorSensor != null )
      colorSensorDetected = colorSensor.isDetected();
    else
      colorSensorFirstDetected = false;
    
    if ( colorSensorDetected == true)
    {
     // System.out.println("detected one cargo");
    }

    if (colorSensorDetected == true && colorSensorFirstDetected == false) 
    {
      colorSensorFirstDetected = true;
      nColorSensorDetectedCount ++;
      System.out.println("start shuffling the cargo, detected time"+nColorSensorDetectedCount);
    }
      
    if (colorSensorFirstDetected == true && nColorSensorDetectedCount < 3) 
    {
      if ( nColorSensorDetectedCount == 1)
      {
        //@todo: currently for auto mode pre-loaded one cargo
        if(Robot.m_bAutoMode==true)
        {
          indexer.runForIntake(6);
        }
        else
        {
          indexer.runForIntake(14);
        }
      }
      else if ( nColorSensorDetectedCount == 2)
       indexer.runForIntake(6);

       counter++;
      // TODO tune for 100
      //For one time position change: note this number depends how fast/smoothly the cargo is shuffled in
      if (counter > 150)
      {
        // At this time, the cargo should be in the indexer, and the indexer should stop

        //reset for the next cargo 
        colorSensorFirstDetected = false;
        colorSensorDetected = false;
        counter = 0;

        indexer.setIndexerPosition();
        indexer.stop();
        System.out.println("stop shuffling the cargo");
      }
    } 
    else 
    {
      indexer.stop();
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

//for the case when the switch is at the end of the indexer, closer to shooter
//  // Called every time the scheduler runs while the command is scheduled.
//  @Override
//  public void execute() {
   
//    //@todo: only consider one detection, ignore all of the following detections.
//    if(bFirstSwitchDetected == false)
//    {
//      switchDetected = switchDetector.isDetected();
//      System.out.println("switch detected: "+switchDetected);
//    }

//    if (switchDetected == true) 
//    {
//      bFirstSwitchDetected = true;
//      indexer.stop();
//    } 
//    else 
//    {
//      colorSensorDetected = colorSensor.isDetected();
//      if ( colorSensorDetected == true)
//      {
//        System.out.println("detected one cargo");
//      }

//      if (colorSensorDetected == true && colorSensorFirstDetected == false) 
//      {
//        colorSensorFirstDetected = true;
//        System.out.println("start shuffling the cargo");
//      }
     
//      if (colorSensorFirstDetected == true) 
//      {
       
//        indexer.runForIntake();
//        counter++;
       
//        // TODO tune for 100
//        if (counter > 200) {
//          // At this time, the cargo should be in the indexer, and the indexer should stop

//          //reset for the next cargo 
//          colorSensorFirstDetected = false;
//          colorSensorDetected = false;
//          counter = 0;
//          indexer.setIndexerPosition();
//          System.out.println("stop shuffling the cargo");
//          indexer.stop();

//        }
//      } else {
//        indexer.stop();
//      }
//    }
//  }