package team3176.robot.subsystems.vision;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class SimPhotonVision extends SubsystemBase{
    // Simulated Vision System.
    
    Transform3d camera2Robot;
    VisionSystemSim simVision = new VisionSystemSim("photonvision");
    PhotonCameraSim simCam;
    AprilTagFieldLayout field;
    PhotonPoseEstimator estimator;
    
    public SimPhotonVision(PhotonCamera c, Transform3d t) {
        camera2Robot = t;
        simCam = new PhotonCameraSim(c, SimCameraProperties.LL2_960_720(),0.05,20);
        simVision.addCamera(simCam, camera2Robot);
        //simVision.addVisionTargets(new VisionTargetSim(t2pose,TargetModel.kTag16h5,2));
        try {
            field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            simVision.addVisionTargets(field);
        }
        catch(Exception e) {
            System.out.println("woops can't load the field");
        }
        estimator = new PhotonPoseEstimator(field, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, c, camera2Robot);
    }

    @Override
    public void periodic() {
        
        Pose2d currentPose = Drivetrain.getInstance().getPoseOdomTrue();
        simVision.update(currentPose);
    }
    
}
