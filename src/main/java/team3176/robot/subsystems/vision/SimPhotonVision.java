package team3176.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class SimPhotonVision extends SubsystemBase{
    // Simulated Vision System.
    
    VisionSystemSim simVision = new VisionSystemSim("photonvision");
    PhotonPoseEstimator estimator;
    public SimPhotonVision(List<PhotonCamera> c, List<Transform3d> t, AprilTagFieldLayout field) {
        for(int i = 0; i < c.size(); i++) {
            Transform3d camera2Robot = t.get(i);
            PhotonCameraSim simCam = new PhotonCameraSim(c.get(i), SimCameraProperties.LL2_1280_720(),0.07,Units.feetToMeters(30));
            simVision.addCamera(simCam, camera2Robot);
        }
        
        //simVision.addVisionTargets(new VisionTargetSim(t2pose,TargetModel.kTag16h5,2));
        simVision.addAprilTags(field);
    }
    public void switchAllaince(AprilTagFieldLayout field) {
        simVision.removeVisionTargets("apriltags");
        simVision.addAprilTags(field);
    }
    @Override
    public void periodic() {
        
        Pose2d currentPose = Drivetrain.getInstance().getSimNoNoisePose();
        simVision.update(currentPose);
    }
    
}
