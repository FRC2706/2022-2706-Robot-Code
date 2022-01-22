package frc.robot.commands.ramseteAuto;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.Config;

public class TranslationScaled extends Translation2d {
    /**
     * Constructs a default Translation2d with x=0 & y=0
     */
    public TranslationScaled() {
        super();
    }
    
    /**
     * Constructs a Translation2d which is scaled based
     * on field scale in Config
     */
    public TranslationScaled(double x, double y) {
        super(x * Config.scaleField, y * Config.scaleField);
    }
}
