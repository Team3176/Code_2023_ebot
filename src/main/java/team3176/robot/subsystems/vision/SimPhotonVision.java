package team3176.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VideoSimUtil;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
    public SimPhotonVision() {
        realCam = new PhotonCamera("camera1");
        simCam = new PhotonCameraSim(realCam, SimCameraProperties.LL2_960_720(),0.05,20);
        simVision.addCamera(simCam, cameratrans);
        //simVision.addVisionTargets(new VisionTargetSim(t2pose,TargetModel.kTag16h5,2));
        try {
            simVision.addVisionTargets(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
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
            for(PhotonTrackedTarget t :realCam.getLatestResult().getTargets()) {
                targets.add(current3d.transformBy(cameratrans).transformBy(t.getBestCameraToTarget()));
            }

            Logger.getInstance().recordOutput("photonvision/targetposes", targets.toArray(new Pose3d[targets.size()]));
        }
        else {
            Logger.getInstance().recordOutput("photonvision/targetposes", new Pose3d[] {});
        }
    }
    
}
