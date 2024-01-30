package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private static PhotonCamera aprilTagCamera;
    private static PhotonPipelineResult aprilTagCamResult;
    private static PhotonTrackedTarget lastValidTarget;
//    private static PhotonCamera visionCamera;
    public static VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }
    private VisionSubsystem() {
        aprilTagCamera = new PhotonCamera(Constants.Vision.aprilTagCamName);
        updateAprilTagResult();
    }
    public void updateAprilTagResult() {
        aprilTagCamResult = aprilTagCamera.getLatestResult();
    }
    public PhotonPipelineResult getLatestAprilTagResult() {
        updateAprilTagResult();
        return aprilTagCamResult;
    }
    public void getTargets() {
        aprilTagCamResult.getTargets();
    }
    public boolean hasTargets() {
        return aprilTagCamResult.hasTargets();
    }
    public boolean isValidTarget(PhotonTrackedTarget target) {
        int id = target.getFiducialId();
        return 1 <= id && id <= Constants.Vision.aprilTagMax;
    }
    // TODO verify that by the end of auto we have lastValidTarget set
    // theres like no way you dont see one at the start of auto maybe I think
    public PhotonTrackedTarget getBestTarget() {
        if (hasTargets()) {
            PhotonTrackedTarget newTarget = aprilTagCamResult.getBestTarget();
            if (isValidTarget(newTarget)) {
                lastValidTarget = newTarget;
            }
        }
        return lastValidTarget;
    }
}