package frc.robot.nettables;

//for NetworkTable
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Config;

import java.util.function.Supplier;

public class VisionCtrlNetTable {

    //For NetworkTables
    //Get the default instance of NetworkTables that was created automatically
    //when your program starts
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //Add a visionControl network table
    private NetworkTable visionControlTable;
    private NetworkTableEntry VisionShutDownEntry;
    private NetworkTableEntry VisionStartUpEntry;
    private static NetworkTable visionOuterPortTable;
    private static NetworkTable visionPowercellTable;

    public static Supplier<Double> distanceToPowerCell;
    public static Supplier<Double> distanceToOuterPort;
    public static Supplier<Double> yawToPowerCell;
    public static Supplier<Double> yawToOuterPort;
    public static Supplier<Double> yawToDiamond;

    //todo: for later integration
    //public static Supplier<Double> angleToOuterPort;

    public VisionCtrlNetTable () {
        visionControlTable = inst.getTable("VisionControl");
        VisionShutDownEntry = visionControlTable.getEntry("ShutDown");
        VisionStartUpEntry = visionControlTable.getEntry("StartUp");

        VisionShutDownEntry.setBoolean(false);
        VisionStartUpEntry.setBoolean(false);

        visionOuterPortTable = inst.getTable(Config.VISION_TABLE_NAME_OUTERPORT);
        distanceToOuterPort = () -> visionOuterPortTable.getEntry(Config.DISTANCE_OUTER_PORT).getDouble(-1);
        yawToOuterPort = () -> visionPowercellTable.getEntry(Config.YAW_OUTER_PORT).getDouble(-99);

        visionPowercellTable = inst.getTable(Config.VISION_TABLE_NAME_POWERCELL);
        distanceToPowerCell = () -> visionPowercellTable.getEntry(Config.DISTANCE_POWERCELL).getDouble(-1);
        yawToPowerCell = () -> visionPowercellTable.getEntry(Config.YAW_POWERCELL).getDouble(-99);

        visionPowercellTable = inst.getTable(Config.VISION_TABLE_NAME_OUTERPORT);
        yawToDiamond = () -> visionPowercellTable.getEntry(Config.YAW_TO_DIAMOND).getDouble(-99);
        
        

    }

    public static void setTapeMode() {
        visionOuterPortTable.getEntry("Tape").setBoolean(true);
    }

    /**
     * Writes a shut down signal to the vision control table.
     *
     */
    public void shutDownVision()
    {
        VisionShutDownEntry.setBoolean(true);
        VisionStartUpEntry.setBoolean(false);

    }

    /**
     * Writes a start up signal to the vision control table.
     *
     */
    public void startUpVision()
    {
        VisionShutDownEntry.setBoolean(false);
        VisionStartUpEntry.setBoolean(true);

    }

}