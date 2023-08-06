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

public class SimVision extends SubsystemBase{
       // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    static double camDiagFOV = 101; // degrees - assume wide-angle camera
    static double camPitch = Units.degreesToRadians(10); // radians
    static double camHeightOffGround = 0.0; // meters
    static double maxLEDRange = 20; // meters
    static int camResolutionWidth = 960; // pixels
    static int camResolutionHeight = 720; // pixels
    static double minTargetArea = 20; // square pixels
    static Transform3d cameratrans = new Transform3d(
        new Translation3d(0.0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0));
    VisionSystemSim simVision = new VisionSystemSim("photonvision");
    PhotonCamera realCam = new PhotonCamera("camera1");
    PhotonCameraSim simCam;
    public SimVision() {
        double targetHeight = Units.inchesToMeters(18.22); // meters
        double tgtXPos = Units.inchesToMeters(610.77);
        double tgtYPos = Units.inchesToMeters(42.19);
        Pose3d farTargetPose =
                new Pose3d(
                        new Translation3d(tgtXPos, tgtYPos, targetHeight),
                        new Rotation3d(0.0, 0.0, 0.0));
        SimCameraProperties props = SimCameraProperties.LL2_960_720();
        simCam = new PhotonCameraSim(realCam, SimCameraProperties.LL2_960_720(),0.05,20);
        simVision.addCamera(simCam, cameratrans);
        VisionTargetSim t1 = new VisionTargetSim(farTargetPose, TargetModel.kTag16h5,1);
        Pose3d t2pose = new Pose3d(1.0, 0.0, 0.8, new Rotation3d());
        //simVision.addVisionTargets(t1);
        //simVision.addVisionTargets(new VisionTargetSim(t2pose,TargetModel.kTag16h5,2));
        try {
            simVision.addVisionTargets(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
        }
        catch(Exception e) {
            System.out.println("woops can't load the field");
        }
        //PhotonCameraSim cam = new PhotonCameraSim(realCam,);
        //simVision.addCamera(null, cameratrasimCamns);
        // SimVisionTarget t1 = new SimVisionTarget(farTargetPose, Units.inchesToMeters(6), Units.inchesToMeters(6), 1);
        // simVision.addSimVisionTarget(t1);
    }

    @Override
    public void periodic() {
        
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        Pose3d current3d = new Pose3d(new Translation3d(currentPose.getX(),currentPose.getY(),0.0), new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));
        simVision.update(currentPose);
        //System.out.println(simVision.getVisionTargets().size());
        var results = realCam.getLatestResult();
        if (results.hasTargets()) {
            ArrayList<Pose3d> targets = new ArrayList<Pose3d>();
            for(PhotonTrackedTarget t :realCam.getLatestResult().getTargets()) {
                targets.add(current3d.transformBy(cameratrans).transformBy(t.getBestCameraToTarget()));
            }

            Logger.getInstance().recordOutput("photonvision/targetposes", targets.toArray(new Pose3d[targets.size()]));
            //Logger.getInstance().recordOutput("photonvision/targetposeraw", new Pose3d().transformBy(cameratrans).transformBy(results.getBestTarget().getBestCameraToTarget()));
        }
        else {
            Logger.getInstance().recordOutput("photonvision/targetpose", new Pose3d[] {});
        }
    }
    
}
