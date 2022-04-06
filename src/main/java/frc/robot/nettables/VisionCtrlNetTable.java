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
    private static NetworkTable visionHubTable;
    private static NetworkTable visionCargoTable;

    public static Supplier<Double> distanceToCargo;
    public static Supplier<Double> yawToCargo;
    public static Supplier<Double> yawToDiamond;

    public static Supplier<Double> distanceToHub;
    public static Supplier<Double> yawToHub;

    //todo: for later integration
    //public static Supplier<Double> angleToOuterPort;

    public VisionCtrlNetTable () {
        visionControlTable = inst.getTable("VisionControl");
        VisionShutDownEntry = visionControlTable.getEntry("ShutDown");
        VisionStartUpEntry = visionControlTable.getEntry("StartUp");

        VisionShutDownEntry.setBoolean(false);
        VisionStartUpEntry.setBoolean(false);

        visionHubTable = inst.getTable(Config.VISION_TABLE_NAME_HUB);
        distanceToHub = () -> visionHubTable.getEntry(Config.DISTANCE_HUB).getDouble(-1);
        yawToHub = () -> visionHubTable.getEntry(Config.YAW_HUB).getDouble(-99);

        visionCargoTable = inst.getTable(Config.VISION_TABLE_NAME_CARGO);
        distanceToCargo = () -> visionCargoTable.getEntry(Config.DISTANCE_CARGO).getDouble(-1);
        yawToCargo = () -> visionCargoTable.getEntry(Config.YAW_CARGO).getDouble(-99);

        yawToDiamond = () -> visionHubTable.getEntry(Config.YAW_TO_DIAMOND).getDouble(-99);
         

    }

    public static void setTapeMode() {
        visionHubTable.getEntry("Tape").setBoolean(true);
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