/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

/**
 * Add your docs here.
 */
public class FeederSubsystem extends ConditionalSubsystemBase {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    //FeederSubsystem is a singleton class as it represents a physical subsystem
    private static FeederSubsystem currentInstance;

    //The motor that drives the feeder
    private static WPI_TalonSRX feederTalon;

    //IR sensor that monitors the indexer
    private static DigitalInput inputSwitch;
    private static DigitalInput outputSwitch;

    //How much to shift the feeder wheel when incrementing
    public static FluidConstant<Double> FEEDERSUBSYSTEM_INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 12_000.0)
                .registerToTable(Config.constantsTable);
    //Max distance at which the robot knows a ball is at the indexer
    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IR_MAX_DISTANCE = new FluidConstant<>("IrMaxDistance", 0)
                .registerToTable(Config.constantsTable);
    // public static FluidConstant<Double> FEEDERSUBSYSTEM_P = new FluidConstant<>("FeederSubsystemP", 0.15)
    //             .registerToTable(Config.constantsTable);
    // public static FluidConstant<Double> FEEDERSUBSYSTEM_I = new FluidConstant<>("FeederSubsystemI", 0.001)
    //             .registerToTable(Config.constantsTable);
    // public static FluidConstant<Double> FEEDERSUBSYSTEM_D = new FluidConstant<>("FeederSubsystemD", 16.5)
    //             .registerToTable(Config.constantsTable);
    // public static FluidConstant<Double> FEEDERSUBSYSTEM_F = new FluidConstant<>("FeederSubsystemF", 0.1)
    //             .registerToTable(Config.constantsTable);
    //Highest speed the motor could reach
    public static FluidConstant<Double> FEEDERSUBSYSTEM_PEAK_OUTPUT = new FluidConstant<>("FeederSubsystemPeakOutput", 1.0)
                .registerToTable(Config.constantsTable);

    // Keep track of how many balls are around the indexer
    private int ballsFeeder = 0;

    private final int kTimeoutMs = 1000;

    private final int kPIDLoopIdx = 0;

    private final double[] arbFF = new double[]{
        Config.FEEDERSUBSYSTEM_ARBFF_ONE,
        Config.FEEDERSUBSYSTEM_ARBFF_TWO,
        Config.FEEDERSUBSYSTEM_ARBFF_THREE,
        Config.FEEDERSUBSYSTEM_ARBFF_FOUR  
    };

    private FeederSubsystem(){

        createCondition("encoderHealthy", SubsystemConditionStates.ALWAYS);

        //Initialize the talon
        feederTalon = new WPI_TalonSRX(Config.FEEDER_SUBSYSTEM_TALON);

        //Initialize the limit switches
        if (Config.FEEDER_SWITCH_INPUT != -1)
            inputSwitch = new DigitalInput(Config.FEEDER_SWITCH_INPUT);

        if (Config.FEEDER_SWITCH_OUTPUT != -1)
            outputSwitch = new DigitalInput(Config.FEEDER_SWITCH_OUTPUT);

        //Configure the talon
        if (checkConditions()){
            feederTalon.configFactoryDefault();
            feederTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
            feederTalon.configNominalOutputForward(0, kTimeoutMs);
            feederTalon.configNominalOutputReverse(0, kTimeoutMs);
            feederTalon.configPeakOutputForward(FEEDERSUBSYSTEM_PEAK_OUTPUT.get(), kTimeoutMs);
            feederTalon.configPeakOutputReverse(-(FEEDERSUBSYSTEM_PEAK_OUTPUT.get()), kTimeoutMs);

           // feederTalon.configAllowableClosedloopError(0, 0, kTimeoutMs);
            feederTalon.config_kF(kPIDLoopIdx, Config.FEEDERSUBSYSTEM_F.get(), kTimeoutMs);
            feederTalon.config_kP(kPIDLoopIdx, Config.FEEDERSUBSYSTEM_P.get(), kTimeoutMs);
            feederTalon.config_kI(kPIDLoopIdx, Config.FEEDERSUBSYSTEM_I.get(), kTimeoutMs);
            feederTalon.config_kD(kPIDLoopIdx, Config.FEEDERSUBSYSTEM_D.get(), kTimeoutMs);
            feederTalon.config_IntegralZone(kPIDLoopIdx, Config.FEEDERSUBSYSTEM_IZONE.get(), kTimeoutMs);
            feederTalon.configAllowableClosedloopError(0, 30, Config.CAN_TIMEOUT_SHORT);
            feederTalon.setSelectedSensorPosition(0, 0, Config.CAN_TIMEOUT_SHORT);

            feederTalon.configMotionCruiseVelocity(Config.FEEDER_MM_CRUISE_VELOCITY);
            feederTalon.configMotionAcceleration(Config.FEEDER_MM_ACCELERATION);
            feederTalon.configMotionSCurveStrength(Config.FEEDER_MM_SCURVE);
        }

    }

    public static void zeroTalon() {
       // if (feederTalon.getControlMode().equals(ControlMode.MotionMagic)) {
        feederTalon.set(ControlMode.MotionMagic, 0);
        feederTalon.stopMotor();
        feederTalon.getSensorCollection().setQuadraturePosition(0, Config.CAN_TIMEOUT_SHORT);
        
        
        
        
        
    }

    public static void init() {
        if (currentInstance == null) {
            currentInstance = new FeederSubsystem();
        }
    }

    public void runAtRPM() {
        double targetVelocity = -1600;
        feederTalon.set(ControlMode.Velocity, targetVelocity);
    }

    public static FeederSubsystem getInstance() {
        init();
        return currentInstance;
    }

    /**
     * Moves the power cells along the feeder track a certain amount
     */
    public void incrementPowerCells(int ticks){
        System.out.println("Incrementing power cells...");
        feederTalon.set(ControlMode.Position, ticks);
    }

    public void runFeeder(){
     //   System.out.println("Feeder at :" + feederTalon.getSelectedSensorPosition());
        feederTalon.set(ControlMode.PercentOutput, -FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    public void runFeeder(double speed){
        //   System.out.println("Feeder at :" + feederTalon.getSelectedSensorPosition());
        feederTalon.set(ControlMode.PercentOutput, speed);
    }

    public void slowReverseFeeder() {
        feederTalon.set(ControlMode.PercentOutput, FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    public void reverseFeeder(){
        feederTalon.set(ControlMode.PercentOutput, -Config.FEEDERSUBSYSTEM_PEAK_OUTPUT.get());
    }

    /**
     * Runs the feeder motor just enough to empty 5 balls out of the feeder
     */
    public void emptyFeeder(){
        feederTalon.setSelectedSensorPosition(0, 0, 10);
        feederTalon.set(ControlMode.Position, 6*Config.FEEDERSUBSYSTEM_INCREMENT_TICKS.get());
    }

    /**
     * Determines if the lift has reached the given setpoint.
     *
     * @return True if the lift has reached the setpoint, false otherwise.
     */
    public boolean doneIncrementing(double lowerLimit) {
        boolean done = false;

        if(feederTalon.getSelectedSensorPosition() <= lowerLimit) {
            done = true;
        }
        return done;
    }

    //GABBY CHANGED to double -> need verification this works as expected
    public double getCurrentPosition() {
        return feederTalon.getSelectedSensorPosition();
    }

    public void periodic() {
        if (Config.FEEDER_SUBSYSTEM_TALON != -1) {
            SmartDashboard.putNumber("Feeder encoder ticks", feederTalon.getSelectedSensorPosition());
            SmartDashboard.putNumber("Feeder RPM", (feederTalon.getSelectedSensorPosition() * 600.0) / 4096);
        }
    }
    public void stopFeeder() {
        feederTalon.stopMotor();
    }


    /**
     * Check if a ball is on the input side of the feeder
     * @return whether a power cell has reached the indexer or not
     */
    public static boolean isBallAtInput(){
        return !inputSwitch.get();
    }

    /**
     * Check if a ball is on the output side of the feeder, the
     * side closest to the shooter
     * @return whether limit switch was it
     */
    public static boolean isBallAtOutput(){
        return !outputSwitch.get();
    }

    /**
     * Get the number of balls around the feeder
     * @return the number of balls around the feeder
     */
    public int getBallsAroundFeeder() {
        return ballsFeeder;
    }

    /**
     * Set the number of balls around the Feeder
     */
    public void setBallsAroundFeeder(int numBalls) {
        ballsFeeder = numBalls;

        // If no balls are around the feeder, 0 the encoder posistion.
        if (ballsFeeder == 0) {
            zeroTalon();

        } else if (ballsFeeder > 5) {
            // TODO: log if it thinks there are more than 5 balls around the feeder
        }
    }

    /**
     * Set the position of the feeder using motion magic
     * @param ticks number of ticks to move to
     */
    public void setFeederPosistion(int ticks) {
        feederTalon.set(ControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward, getArbFF());
    }

    /**
     * Get Arbitrary feedforwards for the feeder depending on how many balls
     * are around the feeder.
     */
    private double getArbFF() {
        int numBalls = getBallsAroundFeeder();
        if (numBalls < 1 || numBalls > Config.FEEDER_MAX_BALLS) {
            numBalls = 1;
        }
        return arbFF[numBalls-1];
    }

    /**
     * Check the error on the posistion/motion magic is it's within
     * a allowed error.
     * 
     * Posistion units are ticks 
     * 
     * Would also work for velocity control mode but units change to 
     * ticks/100ms
     * 
     * @param errorAllowed +/- errorAllowed is within the threshold
     * @return Whether the feeder is at that position
     */
    public boolean isFeederAtPosistion(int errorAllowed) {
        if (Math.abs(feederTalon.getClosedLoopError()) < errorAllowed) {
            return true;
        } 
        return false;
    }

}