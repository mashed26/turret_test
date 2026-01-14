package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final PhotonCamera limelight;
    private final int TARGET_APRILTAG_ID = 9;
    
    // Transform from turret base to camera (adjust based on your robot's measurements)
    private final Transform3d TURRET_TO_CAMERA = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),  // x, y, z in meters
        new Rotation3d(0.0, 0.0, 0.0)      // roll, pitch, yaw in radians
    );
    
    private double currentTurretAngle = 0.0; // Current turret angle in degrees
    private PhotonPipelineResult latestResult = null;
    
    public Vision() {
        // Initialize PhotonVision camera with the name configured in PhotonVision
        limelight = new PhotonCamera("limelight-turret");
        
        limelight.setPipelineIndex(0);
    }
    
    @Override
    public void periodic() {
        // Get all unread results and use the most recent one
        List<PhotonPipelineResult> results = limelight.getAllUnreadResults();
        
        // Update latestResult if we have new data
        if (!results.isEmpty()) {
            // The last result in the list is the most recent
            latestResult = results.get(results.size() - 1);
        }
    }
    
    public void setTurretAngle(double angle) {
        this.currentTurretAngle = angle;
    }
    
    public double getTurretAngle() {
        return currentTurretAngle;
    }
    
    public boolean hasTarget() {
        if (latestResult == null || !latestResult.hasTargets()) {
            return false;
        }
        
        return latestResult.getTargets().stream()
            .anyMatch(target -> target.getFiducialId() == TARGET_APRILTAG_ID);
    }
    
    private Optional<PhotonTrackedTarget> getTarget() {
        if (!hasTarget()) {
            return Optional.empty();
        }
        
        return latestResult.getTargets().stream()
            .filter(target -> target.getFiducialId() == TARGET_APRILTAG_ID)
            .findFirst();
    }
    
    public double getDistanceToTarget() {
        Optional<PhotonTrackedTarget> target = getTarget();
        
        if (target.isEmpty()) {
            return -1.0;
        }
        
        Transform3d cameraToTarget = target.get().getBestCameraToTarget();
        
        // Calculate 2D distance (ignoring vertical component)
        double x = cameraToTarget.getX();
        double y = cameraToTarget.getY();
        
        return Math.sqrt(x * x + y * y);
    }
    
    public double get3DDistanceToTarget() {
        Optional<PhotonTrackedTarget> target = getTarget();
        
        if (target.isEmpty()) {
            return -1.0;
        }
        
        Transform3d cameraToTarget = target.get().getBestCameraToTarget();
        
        // Calculate 3D distance
        double x = cameraToTarget.getX();
        double y = cameraToTarget.getY();
        double z = cameraToTarget.getZ();
        
        return Math.sqrt(x * x + y * y + z * z);
    }
    
    public double getAngleToTarget() {
        Optional<PhotonTrackedTarget> target = getTarget();
        
        if (target.isEmpty()) {
            return 0.0;
        }
        
        Transform3d cameraToTarget = target.get().getBestCameraToTarget();
        
        // Get target position relative to camera
        double targetX = cameraToTarget.getX();
        double targetY = cameraToTarget.getY();
        
        // Account for camera offset from turret center
        double adjustedX = targetX - TURRET_TO_CAMERA.getX();
        double adjustedY = targetY - TURRET_TO_CAMERA.getY();
        
        // Calculate angle in degrees
        // atan2(y, x) gives angle where 0° is forward (+X axis)
        double angleRadians = Math.atan2(adjustedY, adjustedX);
        double angleDegrees = Math.toDegrees(angleRadians);
        
        return angleDegrees;
    }
    
    public double getRequiredTurretAngle() {
        double relativeAngle = getAngleToTarget();
        
        if (!hasTarget()) {
            return currentTurretAngle;
        }
        
        // Add the relative angle to current turret angle
        return currentTurretAngle + relativeAngle;
    }
    
    public Translation2d getTargetPositionRelativeToTurret() {
        Optional<PhotonTrackedTarget> target = getTarget();
        
        if (target.isEmpty()) {
            return null;
        }
        
        Transform3d cameraToTarget = target.get().getBestCameraToTarget();
        
        // Transform from camera coordinates to turret base coordinates
        double x = cameraToTarget.getX() - TURRET_TO_CAMERA.getX();
        double y = cameraToTarget.getY() - TURRET_TO_CAMERA.getY();
        
        return new Translation2d(x, y);
    }
    
    public double getTargetYaw() {
        Optional<PhotonTrackedTarget> target = getTarget();
        return target.map(PhotonTrackedTarget::getYaw).orElse(0.0);
    }
    
    public double getTargetPitch() {
        Optional<PhotonTrackedTarget> target = getTarget();
        return target.map(PhotonTrackedTarget::getPitch).orElse(0.0);
    }
    
    public String getTrackingInfo() {
        if (!hasTarget()) {
            return "No target detected";
        }
        
        return String.format(
            "Target #9 | Distance: %.2fm | Angle: %.1f° | Required Turret: %.1f°",
            getDistanceToTarget(),
            getAngleToTarget(),
            getRequiredTurretAngle()
        );
    }
    
    /**
     * Get the timestamp of the latest result (useful for latency compensation)
     */
    public double getLatestTimestamp() {
        if (latestResult == null) {
            return 0.0;
        }
        return latestResult.getTimestampSeconds();
    }
    
    /**
     * Check how old the latest data is (in seconds)
     */
    public double getDataAge() {
        if (latestResult == null) {
            return Double.MAX_VALUE;
        }
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - latestResult.getTimestampSeconds();
    }
}