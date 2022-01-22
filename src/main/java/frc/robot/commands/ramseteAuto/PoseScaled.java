package frc.robot.commands.ramseteAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.Config;

public class PoseScaled extends Pose2d {
    /**
     * Constructs a default Pose2d with x=0 & y=0 & rot=0
     */
    public PoseScaled() {
        super();
    }

    /**
     * Constructs a scaled Pose2d based on field scale
     * Unit of rotation is degrees
     */
    public PoseScaled(double x, double y, double deg) {
        super(x * Config.scaleField, y * Config.scaleField, Rotation2d.fromDegrees(deg));
    }
}
