package frc.robot.commands.ramseteAuto;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.config.Config;
import frc.robot.nettables.VisionCtrlNetTable;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

    /**
     * -------------------
     * 
     * 
     * To add a vision type to this class, follow the below steps
     * 
     * 
     * Add an enum entry for the vision type.
     * 
     * Switch statements should throw warnings. Go through and add
     *    add support for your need vision type.
     * 
     * Write a new calc method for that vision type. Though this could return any pose
     *    you want, it was setup to be poses that have their origin on the field origin.
     *    To write this calc method you'll need to read the other ones to understand it.
     *    It can also handle a Translation2d where the Rotation2d of the Pose2d is just 0 
     *    and the Rotation2d is ignored where this method is called.
     *      
     * 
     * ------------------
     */


public class VisionPose {
    // Important boolean to set
    private final boolean usePerpendicularAngle = Config.useVisionPerpendicularAngle;


    // static variable single_instance of type Singleton 
    private static VisionPose single_instance = null;

    // static method to create instance of Singleton class 
    public static VisionPose getInstance() 
    { 
        if (single_instance == null) 
            single_instance = new VisionPose(); 
  
        return single_instance; 
    }

    // Which field is set up, default to general until told otherwise
    private Field field = Field.General;

    // Camera Poses from centre
    final Pose2d diamondTapeCamera = Config.diamondTapeCamera;
    final Pose2d middleOfConesCamera = Config.middleOfConesCameraLocation;

    private final double defaultValue = Config.VISION_NO_TARGET_CODE;

    NetworkTableEntry tTargetDistanceToTarget;
    NetworkTableEntry tTargetAngleAtRobot;
    NetworkTableEntry tTargetAngleAtTarget;

    NetworkTableEntry diamondTapeDistanceToTarget;
    NetworkTableEntry diamondTapeAngleAtRobot;
    NetworkTableEntry diamondTapeAngleAtTarget;

    NetworkTableEntry middleOfConesDistanceToTarget;
    NetworkTableEntry middleOfConesAngleAtRobot;
    NetworkTableEntry middleOfConesAngleAtTarget;

    private VisionPose() {

        // Test setup, T tape
        var table = NetworkTableInstance.getDefault().getTable("Vision2017"); 
        tTargetDistanceToTarget = table.getEntry("tapeDistance");
        tTargetAngleAtRobot = table.getEntry("tapeYaw");
        tTargetAngleAtTarget = table.getEntry("tapeTargetYaw"); // NOT CORRECT


        // See spreadsheet link below for name of entries, 
        // sheet name "Vision-Robot Code Network Table Interface"
        // https://docs.google.com/spreadsheets/d/1PnGQ9u9Dbw-_QH5AF-kSOHGbEtmQRehaz8KhUy-qtFo/edit?usp=sharing

        // Diamond Tape
        table = NetworkTableInstance.getDefault().getTable("MergeVisionPipelinePi22"); 

        diamondTapeDistanceToTarget = table.getEntry("YawToDiamond");
        diamondTapeAngleAtRobot = table.getEntry("DistanceToDiamond");
        diamondTapeAngleAtTarget = table.getEntry("RotationAngleToDiamondPerpendicular");

        // Middle of Cones
        table = NetworkTableInstance.getDefault().getTable("MergeVisionPipelinePi21");

        middleOfConesDistanceToTarget = table.getEntry("DistanceToTwoConeMidpoint");
        middleOfConesAngleAtRobot = table.getEntry("YawToTwoConeMidpoint");
        middleOfConesAngleAtTarget = table.getEntry("RotationAngleToTwoConePerpendicular");
    }

    public enum VisionType {
        // 2021 game
        MiddleOfCones,
        DiamondTape, 

        // Other
        TPracticeTarget;
        
    }

    public enum Field {
        // Non specific
        General,

        // 2021 Game, Infinite Recharge At Home
        BarrelRacing,
        Slalom, 
        Bounce;

    }

    /**
     * Init the Vision 
     */
    public void initVision(VisionType visionType) {
        switch (visionType) {
            case MiddleOfCones:

                break;

            case DiamondTape:

                break;

            case TPracticeTarget:
                VisionCtrlNetTable.setTapeMode(); 
                break;
            
            
        }
    }

    /**
     * Get the Trajectory Config object needed to construct a trajectory
     * 
     * Velocity Units are in meters per second
     */
    public TrajectoryConfig getTrajConfig(double startVelocity, double endVelocity, boolean isReversed) {
        return Config.trajectoryConfig.setStartVelocity(startVelocity).setEndVelocity(endVelocity).setReversed(isReversed);
    }

    /**
     * Get the Trajectory Config object needed to construct a trajectory
     * 
     * Velocity Units are in meters per second
     * Determines whether to drive in reverse based on Vision Type
     */
    public TrajectoryConfig getTrajConfig(double startVelocity, double endVelocity, VisionType visionType) {
        return getTrajConfig(startVelocity, endVelocity, getReversed(visionType));
    }

    public boolean getReversed(VisionType visionType) {
        switch (visionType) {
        case MiddleOfCones:
            return true;

        case DiamondTape:
            return true;

        case TPracticeTarget:
            return false;
        }

        return false;
    }

    /**
     * Returns a Pose relative to field of a given vision target.
     * If vision type can only calculate translation it will return
     * a pose with a rotation of 0. 
     * 
     * Null means no calculation was possible
     */
    public Pose2d getTargetPose(VisionType visionType) {
        switch (visionType) {
            case MiddleOfCones:
                return calcMiddleOfCones();
    
            case DiamondTape:
                return calcDiamondTape();

            case TPracticeTarget:
                return calcTPracticeTarget();

            // If vision type doesn't exist return null
            default:
                return null;
        }
    }

    /**
     * -------------------
     * 
     * 
     * 
     * 
     * Next section of code defines helper methods to do the pose math
     * 
     * 
     * 
     * 
     * ------------------
     */

    private double feetToMeters(double feet) {
        return feet * Config.METERS_IN_ONE_FOOT;
    }
    /**
     * Transform Relative Pose with origin at centre of robot to 
     * field Pose with origin of wherever the Odometry reset it's origin to
     */
    private Pose2d transformPoseToField(Pose2d relativePose) {
        Transform2d transformToFieldCoordinateSystem = new Transform2d(relativePose.getTranslation(), relativePose.getRotation());
        return DriveBaseHolder.getInstance().getPose().transformBy(transformToFieldCoordinateSystem); 
    }

    private Translation2d transformTranslationToField(Translation2d relativeTranslation) {
        Pose2d odometryPose = DriveBaseHolder.getInstance().getPose();
        Translation2d fieldTranslation = relativeTranslation.rotateBy(odometryPose.getRotation());
        fieldTranslation = odometryPose.getTranslation().plus(fieldTranslation);
        return fieldTranslation;
    }

    /**
     * Transform camera location to centre of robot
     */
    private Pose2d transformCameraToCentre(Pose2d relativePose, Pose2d cameraPose) {
        // Transform the camera pose by the relative pose. Done in this order since we want the relative
        // pose to be transformed into the coordinate frame (origin centre of robot) of cameraPose
        return cameraPose.plus(new Transform2d(relativePose.getTranslation(), relativePose.getRotation()));
    }

    /**
     * Transform camera location to centre of robot
     */
    private Translation2d transformCameraToCentre(Translation2d relativeCameraTranslation, Pose2d cameraPose) {
        return transformCameraToCentre(new Pose2d(relativeCameraTranslation, new Rotation2d(0)), cameraPose).getTranslation();
    }


    /**
     * Transform a pose off a wall
     */
    public Pose2d transformPoseOffWall(Pose2d pose, double distanceOffWall) {
        Rotation2d rotation = pose.getRotation();
        Translation2d offsetTranslation = new Translation2d(rotation.getCos() * distanceOffWall,
                rotation.getSin() * distanceOffWall);
        return pose.transformBy(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    /**
     * Transform a translation off a wall in a certain direction
     */
    public Translation2d transformTranslationOffWall(Translation2d translation, double distanceOffWall, Rotation2d direction) {
        Pose2d pose = transformPoseOffWall(new Pose2d(translation, direction), distanceOffWall);
        return pose.getTranslation();
    }

    /**
     * Transforms a pose in any direction.
     * Pose will be moved based on x, y and rotation components of deltaPose
     * 
     * @return Transformed Pose
     */
    public Pose2d transformPose(Pose2d pose, Pose2d deltaPose) {
        return pose.plus(new Transform2d(deltaPose.getTranslation(), deltaPose.getRotation()));
    }

    /**
     * Rotate a pose
     * @param pose
     * @param deltaRotation
     * @return
     */
    public Pose2d rotatePose(Pose2d pose, Rotation2d deltaRotation) {
        return pose.plus(new Transform2d(new Translation2d(), deltaRotation));
    }

    
    /**
     * Given the calculated target location and a known target location, checks if
     * they match up.
     * 
     * Null means they don't match up
     * 
     * If usePerpendicularAngle is set to false this method will take the angle from desiredFieldPose
     * and overwrite the angle of visionFieldPose (which was calculated using the perpendicular angle).
     * This means the robot is reliant on the gyro to accurate, which is not the case over long periods
     * of time.
     * 
     * The rotation target & field pose must be the rotation the robot should be at that pose when it
     * drives through the pose. If its backwards then its 180 deg opposite of what it would be forwards.
     * 
     * @param knownTarget Previously found targets
     * @param visionFieldPose calculated pose with field origin
     * @param targetFieldPose known pose of target with field origin
     * @return Pose2d calculated pose. Null means it doesn't match this target
     */
    private Pose2d checkKnownTarget(Pose2d knownTarget, Pose2d visionFieldPose, PoseScaled targetFieldPose) {
        // If a known target has already been found pass it through
        if (knownTarget != null) 
            return knownTarget;

        double distanceBetween = visionFieldPose.getTranslation().getDistance(targetFieldPose.getTranslation());

        // If the distance between is less than the allowable error its a known target
        if (Math.abs(distanceBetween) < Config.ALLOWABLE_VISION_ODOMETRY_ERROR) {

            // Check whether to use gyro to handle perpendicular angle
            if (usePerpendicularAngle == false) {
                // Set the calculated pose to what it should be on field oriented dimensions. Reliant on gyro.
                visionFieldPose = new Pose2d(visionFieldPose.getTranslation(), targetFieldPose.getRotation());
            }
            return visionFieldPose;
        }
        return null;
    }

    /**
     * This parameter list has the same functionality as the regular checkKnownTarget. 
     * See that method for details on it.
     * 
     * If the target is a known target this method will apply a desired offset to the field pose.
     * The offset is calculated between the known location of the target and the desired location.
     * 
     * The rotation target & field pose must be the rotation the robot should be at that pose when it
     * drives through the pose. If its backwards then its 180 deg opposite of what it would be forwards.
     * 
     * @param knownTarget Whether a target has been matched yet
     * @param visionFieldPose Pose with a field origin calculated from vision data
     * @param targetFieldPose Known location of a target with a field origin
     * @param desiredFieldPose The desired pose with a field origin. 
     * @return Pose2d calculated and offset pose. Null means it doesn't match this target
     */
    private Pose2d checkKnownTarget(Pose2d knownTarget, Pose2d visionFieldPose, PoseScaled targetFieldPose, PoseScaled desiredFieldPose) {
        // If a known target has already been found pass it through
        if (knownTarget != null) {
            return knownTarget;
        }

        // Check if it's a known target
        visionFieldPose = checkKnownTarget(knownTarget, visionFieldPose, targetFieldPose); 

        // If it's a known target, do some additional math to visionFieldPose
        if (visionFieldPose != null) {

             // The transformation is the delta between the known target location and desired pose location
            visionFieldPose = transformPose(visionFieldPose, targetFieldPose.relativeTo(desiredFieldPose));

            // Check whether to use gyro to handle perpendicular angle
            // This method wants the desired field angle to be the angle
            if (usePerpendicularAngle == false) {
                // Set the calculated pose to what it should be on field oriented dimensions. Reliant on gyro.
                visionFieldPose = new Pose2d(visionFieldPose.getTranslation(), desiredFieldPose.getRotation());
            }

            return visionFieldPose;
        }
        
        // VisionFieldPose is null so pass it on
        return null;
    }

    /**
     * -------------------
     * 
     * Next section of code defines methods that do code for a specific vision target, each with different values
     * 
     * ------------------
     */

    private Pose2d calcMiddleOfCones() {
        double distanceToTarget = middleOfConesDistanceToTarget.getDouble(defaultValue);
        double angleAtRobot = middleOfConesAngleAtRobot.getDouble(defaultValue) * Config.VISION_FLIP_ANGLE;
        double angleAtTarget = middleOfConesAngleAtTarget.getDouble(defaultValue) * Config.VISION_FLIP_ANGLE;

        if (Config.useVisionPerpendicularAngle == false) {
            angleAtTarget = 0;
        }

        // Check for "code" that means no data available
        if ((int) distanceToTarget == -99 || (int) angleAtRobot == -99 || (int) angleAtTarget == -99)
            return null;

        // Check that distance is in a reasonable range
        if (distanceToTarget <= 0.2 || distanceToTarget > 6.0)
            return null;
        
        // Check that angle is in a reasonable range
        if (Math.abs(angleAtRobot) > 40)
            return null;


        // Change distance to meters
        distanceToTarget = feetToMeters(distanceToTarget);

        /** Calculate Relative Pose (origin defined as centre of robot) */

        // Calculate relative pose to camera (origin defined as centre of camera, direction camera is facing as angle 0)
        Rotation2d angle = Rotation2d.fromDegrees(angleAtRobot);
        Translation2d translation = new Translation2d(distanceToTarget, angle); // distance*cosTheta, distance*sinTheta
        Pose2d relativePose = new Pose2d(translation, Rotation2d.fromDegrees((angleAtRobot - angleAtTarget) * Config.VISION_FLIP_PERPENDICULAR_ANGLE));

        // Calculate relative pose to robot centre (origin defined as centre of robot, direction towards front of robot as angle 0)
        relativePose = transformCameraToCentre(relativePose, middleOfConesCamera);

        /** Calculate Field Pose (origin defined as "field" origin) */
        // Odometry knowns centre of robot location  (origin of field),
        // vision knowns target location (origin centre of robot)
        Pose2d fieldPose = transformPoseToField(relativePose);
        
        return fieldPose;

        /**
         * MOVED INTO VISION TRAJECTORY COMMANDS

        // TODO: Move this into trajectory commands
        // If the robot is driving backwards the rotation of the angle also needs to be backwards
        if (getReversed(VisionType.DiamondTape)) {
            fieldPose = new Pose2d(fieldPose.getTranslation(), fieldPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        }

        // TODO: Move this into trajectory commands

        // Stores the calculated pose if it's near a known target
        Pose2d knownTarget = null;
        
        switch (field) {
            case General: break;
            case BarrelRacing: break;
            case Slalom: {
                // May want to go through these cones at an angle which means added a desired field 
                // pose with a different rotation component.

                // Closer Middle of Cones
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled()); //FILLOUT

                // Farther Middle of Cones
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled()); //FILLOUT

                // End zone Middle of Cones
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled()); //FILLOUT

                break;
            }
            case Bounce: {
                // First Middle of Cones (D3 to D5)
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled(3.0, -3.1, 90), new PoseScaled(3.0, -3.1, 90+25)); //PATHWEAVER-DATA

                // Second Middle of Cones for second starred marker (B5 to B7)
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled(4.58, -1.56, -90), new PoseScaled(4.58, -1.11, -90)); //PATHWEAVER-DATA

                // End zone Middle of Cones (B10 to D10)
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled(7.58, -2.28, 180)); //PATHWEAVER-DATA 

                break;
            }
        }
        return knownTarget;

        */
    }

    private Pose2d calcDiamondTape() {
        double distanceToTarget = diamondTapeDistanceToTarget.getDouble(defaultValue);
        double angleAtRobot = diamondTapeAngleAtRobot.getDouble(defaultValue) * Config.VISION_FLIP_ANGLE;
        double angleAtTarget = diamondTapeAngleAtTarget.getDouble(defaultValue) * Config.VISION_FLIP_ANGLE;

        if (Config.useVisionPerpendicularAngle == false) {
            angleAtTarget = 0;
        }

        // Check for "code" that means no data available
        if ((int) distanceToTarget == -99 || (int) angleAtRobot == -99 || (int) angleAtTarget == -99)
            return null;

        // Check that distance is in a reasonable range
        if (distanceToTarget <= 0.2 || distanceToTarget > 6.0)
            return null;
        
        // Check that angle is in a reasonable range
        if (Math.abs(angleAtRobot) > 40)
            return null;

        // Change distance to meters
        distanceToTarget = feetToMeters(distanceToTarget);

        // Calculate Relative Pose (origin defined as centre of robot)
        Rotation2d angle = Rotation2d.fromDegrees(angleAtRobot);
        Translation2d translation = new Translation2d(distanceToTarget, angle); // distance*cosTheta, distance*sinTheta
        Pose2d relativePose = new Pose2d(translation, Rotation2d.fromDegrees((angleAtRobot - angleAtTarget) * Config.VISION_FLIP_PERPENDICULAR_ANGLE));

        // Change origin from camera location to robot origin, aka centre of robot.
        relativePose = transformCameraToCentre(relativePose, diamondTapeCamera);

        // Use odometry to calculate pose with a nominal field origin 
        // Odometry knowns centre of robot location (origin defined as "field" origin)
        Pose2d fieldPose = transformPoseToField(relativePose);

        return fieldPose;
        /**
         * FUNCTIONALITY MOVED INTO COMMANDS THAT USE VISIONPOSE
         * 
        // If the robot is driving backwards the rotation of the angle also needs to be backwards
        if (getReversed(VisionType.DiamondTape)) {
            fieldPose = new Pose2d(fieldPose.getTranslation(), fieldPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        }

        
        // Stores the calculated pose if it's near a known target
        Pose2d knownTarget = null;

        switch (field) {
            case General: break;
            case BarrelRacing: {
                // First Diamond
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled(), new PoseScaled()); //FILLOUT
                 
                // Second Diamond
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled(), new PoseScaled()); //FILLOUT

                break;
            }
            case Slalom: break;
            case Bounce: {
                // First Diamond for third starred marker
                knownTarget = checkKnownTarget(knownTarget, fieldPose, new PoseScaled(6.872, -4.6, 90), new PoseScaled(6.872, -1, 90)); //PATHWEAVER-DATA

                break;
            }
        }
        return knownTarget;

        */


    }

    private Pose2d calcTPracticeTarget() {
        double distanceToTarget = tTargetDistanceToTarget.getDouble(-99);
        double angleAtRobot = tTargetAngleAtRobot.getDouble(-99);
        double angleAtTarget = tTargetAngleAtTarget.getDouble(-99);

        if (Config.useVisionPerpendicularAngle == false) {
            angleAtTarget = 0;
        }
        
        // Check for "code" that means no data available
        if ((int) distanceToTarget == -99 || (int) angleAtRobot == -99 || (int) angleAtTarget == -99)
            return null;

        // Check that distance is in a reasonable range
        if (distanceToTarget <= 0.2 || distanceToTarget > 6.0)
            return null;
        
        // Check that angle is in a reasonable range
        if (Math.abs(angleAtRobot) > 40)
            return null;

        // Change distance to meters
        // distanceToTarget = feetToMeters(distanceToTarget);

        // Calculate Relative Pose
        Rotation2d angle = Rotation2d.fromDegrees(angleAtRobot);
        Translation2d translation = new Translation2d(distanceToTarget, angle); // math -> distance*cosTheta, distance*sinTheta
        Pose2d relativePose = new Pose2d(translation, Rotation2d.fromDegrees((angleAtRobot - angleAtTarget) * Config.VISION_FLIP_PERPENDICULAR_ANGLE)); // Flip angle?

        // Change origin from camera location to robot origin, aka centre of robot.
        relativePose = transformCameraToCentre(relativePose, diamondTapeCamera);

        System.out.println("RelativePose is " + relativePose.toString());
        // Use odometry to calculate pose with a nominal field origin 
        Pose2d fieldPose = transformPoseToField(relativePose);

        return fieldPose;
    }


    public void test() {
        // DriveBase drive = DriveBaseHolder.getInstance();

        // Translation2d relative = new Translation2d(1,0);
        // drive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        // System.out.printf("\n\n\n");
        // System.out.println("Odometry: " + drive.getPose());
        // System.out.printf("Rel: %s, Field: %s \n", relative.toString(), transformTranslationToField(relative));
        // System.out.printf("\n\n\n");

        
        tTargetDistanceToTarget.setNumber(1.04);
        tTargetAngleAtRobot.setNumber(0);
        tTargetAngleAtTarget.setNumber(0);
        DriveBaseHolder.getInstance().resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        System.out.println("Odometry: " + DriveBaseHolder.getInstance().getPose().toString());
        System.out.println("Vision Pose: " + getTargetPose(VisionType.TPracticeTarget).toString());

        tTargetDistanceToTarget.setNumber(1.06);
        tTargetAngleAtRobot.setNumber(-20);
        tTargetAngleAtTarget.setNumber(0);
        DriveBaseHolder.getInstance().resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(20)));
        System.out.println("Odometry: " + DriveBaseHolder.getInstance().getPose().toString());
        System.out.println("Vision Pose: " + getTargetPose(VisionType.TPracticeTarget).toString());

    }

    /**
     * Getter and Setters
     */

    public Field getField() {
        return field;
    }

    public void setField(Field field) {
        this.field = field;
    }
}
