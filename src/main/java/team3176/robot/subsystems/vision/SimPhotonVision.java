package team3176.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.estimation.PNPResults;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VideoSimUtil;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class SimPhotonVision extends SubsystemBase{
    // Simulated Vision System.
    
    double camPitch = Units.degreesToRadians(10); // radians
    double camHeightOffGround = 0.8; // meters
    Transform3d cameratrans = new Transform3d(
        new Translation3d(0.0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0));
    VisionSystemSim simVision = new VisionSystemSim("photonvision");
    PhotonCamera realCam;
    PhotonCameraSim simCam;
    AprilTagFieldLayout field;
    HashMap<Integer,VisionTargetSim> targetLookup = new HashMap<>();
    
    public SimPhotonVision() {
        realCam = new PhotonCamera("camera1");
        simCam = new PhotonCameraSim(realCam, SimCameraProperties.LL2_960_720(),0.05,20);
        simVision.addCamera(simCam, cameratrans);
        //simVision.addVisionTargets(new VisionTargetSim(t2pose,TargetModel.kTag16h5,2));
        try {
            field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            simVision.addVisionTargets(field);
            for (VisionTargetSim tgt: simVision.getVisionTargets()) {
                targetLookup.put(tgt.fiducialID, tgt);
            }
        }
        catch(Exception e) {
            System.out.println("woops can't load the field");
        }
    }

    @Override
    public void periodic() {
        
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        Pose3d current3d = new Pose3d(currentPose);
        simVision.update(currentPose);
        var results = realCam.getLatestResult();
        if (results.hasTargets()) {
            ArrayList<Pose3d> targets = new ArrayList<Pose3d>();
            ArrayList<Pose3d> estimates = new ArrayList<Pose3d>();
            ArrayList<Translation3d> vertices = new ArrayList<>();
            ArrayList<TargetCorner> corners = new ArrayList<>();
            for(PhotonTrackedTarget t :realCam.getLatestResult().getTargets()) {
                targets.add(current3d.transformBy(cameratrans).transformBy(t.getBestCameraToTarget()));
                estimates.add(PhotonUtils.estimateFieldToRobotAprilTag(t.getBestCameraToTarget(), field.getTagPose(t.getFiducialId()).get() , cameratrans.inverse()));
                vertices.addAll(targetLookup.get(t.getFiducialId()).getFieldVertices());
                corners.addAll(t.getDetectedCorners());
            }
            
            PNPResults pnpresult = OpenCVHelp.solvePNP_SQPNP(simCam.prop.getIntrinsics(), simCam.prop.getDistCoeffs(), vertices , corners);
            Transform3d camera2target = pnpresult.best;
            Pose3d est = new Pose3d(camera2target.inverse().getTranslation(),camera2target.inverse().getRotation()).transformBy(cameratrans.inverse());
            Logger.getInstance().recordOutput("photonvision/targetposes", targets.toArray(new Pose3d[targets.size()]));
            Logger.getInstance().recordOutput("photonvision/poseEstimates", estimates.toArray(new Pose3d[estimates.size()]));
            Logger.getInstance().recordOutput("photonvision/poseEstimateCustom", est);
        }
        
        else {
            Logger.getInstance().recordOutput("photonvision/targetposes", new Pose3d[] {});
            Logger.getInstance().recordOutput("photonvision/poseEstimates", new Pose3d[] {});
        }
    }
    
}
