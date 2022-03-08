// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IndexerSubSystem;
import frc.robot.subsystems.SwitchSubsystem;

public class IndexerCargo extends CommandBase {

  private IndexerSubSystem indexer;
  private ColorSensorSubsystem colorSensor;
  private SwitchSubsystem switchDetector;
  public boolean switchDetected;
  public boolean colorSensorDetected;
  public boolean colorSensorFirstDetected = false;
  public int counter = 0;

  /** Creates a new IndexerCargo. */
  public IndexerCargo() {

    indexer = IndexerSubSystem.getInstance();
    colorSensor = ColorSensorSubsystem.getInstance();
    switchDetector = new SwitchSubsystem(Config.INDEXER_SWITCH);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(colorSensor);
    addRequirements(switchDetector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    indexer.setIndexerPosition();
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    switchDetected = switchDetector.isDetected();

    if (switchDetected == true) {
      indexer.stop();
    } else {
      colorSensorDetected = colorSensor.isDetected();
      if ( colorSensorDetected == true)
      {
      //  System.out.println("detected one cargo");
      }

      if (colorSensorDetected == true && colorSensorFirstDetected == false) {
        colorSensorFirstDetected = true;
        System.out.println("start shuffling the cargo");
      }
      
      if (colorSensorFirstDetected == true) {
        
        indexer.runForIntake();
        counter++;
        
        // TODO tune for 100
        if (counter > 200) {
          // At this time, the cargo should be in the indexer, and the indexer should stop

          //reset for the next cargo 
          colorSensorFirstDetected = false;
          colorSensorDetected = false;
          counter = 0;
          indexer.setIndexerPosition();
          System.out.println("stop shuffling the cargo");
          indexer.stop();

        }
      } else {
        indexer.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
