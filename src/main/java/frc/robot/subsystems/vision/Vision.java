package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.vision.LimelightVision;
import com.teamscreamrobotics.vision.LimelightVision.Limelight;
import com.teamscreamrobotics.vision.LimelightHelpers;

import java.util.function.Supplier;

public class Vision extends SubsystemBase {
    private final Limelight limelight;
    private final int TARGET_APRILTAG_ID = 9;
    
    // Camera position relative to turret base
    private final Pose3d CAMERA_RELATIVE_POSITION = new Pose3d(
        new Translation3d(Length.fromFeet(7).getMeters(), 0.0, 0.5),  // x, y, z in meters
        new Rotation3d(0.0, 0.785398163, 0.0)      // roll, pitch, yaw in radians
    );
    
    private double currentTurretAngle = 0.0; // Current turret angle in degrees
    private Supplier<Pose2d> robotPoseSupplier;
    
    public Vision(Supplier<Pose2d> robotPoseSupplier) {
        // Initialize Limelight with name and relative position
        limelight = new Limelight("limelight-turret", CAMERA_RELATIVE_POSITION);
        this.robotPoseSupplier = robotPoseSupplier;
        
        // Set the priority tag to our target AprilTag
        LimelightVision.setPriorityTagID(TARGET_APRILTAG_ID, limelight);
    }
    
    @Override
    public void periodic() {
        // No need to manually fetch results - LimelightHelpers handles this
        if(DriverStation.isDisabled()){
            LimelightVision.setThrottle(200, limelight);
        }
        else {LimelightVision.setThrottle(0, limelight);}

        SmartDashboard.putNumber("angle", getDesiredAngle());
    }
    
    public void setTurretAngle(double angle) {
        this.currentTurretAngle = angle;
    }
    
    public double getTurretAngle() {
        return currentTurretAngle;
    }
    
    public boolean hasTarget() {
        return LimelightVision.getTV(limelight);
    }
    
    /**
     * Get the desired turret angle to point at the AprilTag
     * Use this with turret.moveToAngleCommand(vision.getDesiredAngle())
     */
    public double getDesiredAngle() {
        if (!hasTarget()) {
            return currentTurretAngle; // Hold current position if no target
        }
        
        // Get robot pose
        Pose2d robotPose = robotPoseSupplier.get();
        
        // Get target position in robot space
        Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelight.name());
        
        // Project to 2D
        Translation2d targetPositionRobotSpace = new Translation2d(
            targetPoseRobotSpace.getX(),
            targetPoseRobotSpace.getY()
        );
        
        // Calculate angle from robot center to target
        double angleToTargetRadians = Math.atan2(
            targetPositionRobotSpace.getY(),
            targetPositionRobotSpace.getX()
        );
        double angleToTargetDegrees = Math.toDegrees(angleToTargetRadians);
        
        // The turret angle is relative to the robot's forward direction
        return angleToTargetDegrees;
    }
    
    /**
     * Get the field-relative position of the AprilTag target
     */
    public Pose2d getTargetFieldPosition() {
        if (!hasTarget()) {
            return null;
        }
        
        // Get the robot's current pose
        Pose2d robotPose = robotPoseSupplier.get();
        
        // Get target pose in robot space from Limelight
        Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelight.name());
        
        // Convert to 2D and transform to field coordinates
        Translation2d targetTranslation = new Translation2d(
            targetPoseRobotSpace.getX(),
            targetPoseRobotSpace.getY()
        );
        
        // Rotate by robot heading and add to robot position
        Translation2d targetFieldTranslation = targetTranslation
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());
        
        // Get target rotation (yaw only for 2D)
        double targetYaw = targetPoseRobotSpace.getRotation().getZ();
        Rotation2d targetRotation = robotPose.getRotation().plus(Rotation2d.fromRadians(targetYaw));
        
        return new Pose2d(targetFieldTranslation, targetRotation);
    }
    
    /**
     * Get the error between current turret angle and desired angle
     */
    public double getTurretAngleError() {
        if (!hasTarget()) {
            return 0.0;
        }
        
        double desiredAngle = getDesiredAngle();
        double error = desiredAngle - currentTurretAngle;
        
        // Normalize to -180 to 180 degrees
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        return error;
    }
    
    /**
     * Check if the turret is aimed at the target (within tolerance)
     */
    public boolean isTurretAimed(double toleranceDegrees) {
        return hasTarget() && Math.abs(getTurretAngleError()) < toleranceDegrees;
    }
    
    public double getDistanceToTarget() {
        if (!hasTarget()) {
            return -1.0;
        }
        
        // Get 3D position of target in robot space
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(limelight.name());
        Translation3d targetPos = targetPose.getTranslation();
        
        // Calculate 2D distance (ignoring vertical component)
        double x = targetPos.getX();
        double y = targetPos.getY();
        
        return Math.sqrt(x * x + y * y);
    }
    
    public double get3DDistanceToTarget() {
        if (!hasTarget()) {
            return -1.0;
        }
        
        return LimelightVision.get3D_DistanceToTarget(limelight).getMeters();
    }
    
    /**
     * Get the angle from the robot to the target (robot-relative)
     */
    public double getAngleToTarget() {
        return getDesiredAngle();
    }
    
    public Translation2d getTargetPositionRelativeToRobot() {
        if (!hasTarget()) {
            return null;
        }
        
        // Get target position in robot space
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(limelight.name());
        
        return new Translation2d(targetPose.getX(), targetPose.getY());
    }
    
    public double getTargetYaw() {
        if (!hasTarget()) {
            return 0.0;
        }
        return LimelightVision.getTX(limelight);
    }
    
    public double getTargetPitch() {
        if (!hasTarget()) {
            return 0.0;
        }
        return LimelightVision.getTY(limelight);
    }
    
    public String getTrackingInfo() {
        if (!hasTarget()) {
            return "No target detected";
        }
        
        return String.format(
            "Target #9 | Distance: %.2fm | Current: %.1f° | Desired: %.1f° | Error: %.1f°",
            getDistanceToTarget(),
            currentTurretAngle,
            getDesiredAngle(),
            getTurretAngleError()
        );
    }
    
    /**
     * Get the timestamp of the latest result (useful for latency compensation)
     */
    public double getLatestTimestamp() {
        // Calculate timestamp by subtracting latency from current time
        double latencySeconds = LimelightVision.getLatency(limelight) / 1000.0;
        return Timer.getFPGATimestamp() - latencySeconds;
    }
    
    /**
     * Check how old the latest data is (in seconds)
     */
    public double getDataAge() {
        // Limelight latency represents how old the data is
        return LimelightVision.getLatency(limelight) / 1000.0;
    }
    
    /**
     * Set LED mode for the Limelight
     */
    public void setLEDMode(LimelightVision.LEDMode mode) {
        LimelightVision.setLEDMode(mode, limelight);
    }
    
    /**
     * Set the pipeline index
     */
    public void setPipeline(int index) {
        LimelightVision.setPipeline(index, limelight);
    }
    
    /**
     * Get the current pipeline index
     */
    public int getCurrentPipeline() {
        return LimelightVision.getCurrentPipeline(limelight);
    }
}