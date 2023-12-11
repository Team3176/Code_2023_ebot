package team3176.robot.subsystems.vision;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class PhotonVisionSystem extends SubsystemBase{
    static double camPitch = Units.degreesToRadians(-20); // radians
    static double camHeightOffGround = Units.inchesToMeters(8); // meters
    public static final Transform3d camera2Robot = new Transform3d(
        new Translation3d(0.0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0));
    private PhotonCamera realCam;
    AprilTagFieldLayout field;
    PhotonPoseEstimator estimator;
    private static PhotonVisionSystem instance;
    private SimPhotonVision simInstance;
    EstimatedRobotPose currentEstimate;
    private PhotonVisionSystem() {
        realCam = new PhotonCamera("camera1");
        try {
            field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } else {
                field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            }
        }
        catch(Exception e) {
            System.out.println("woops can't load the field");
        }
        
        if(Constants.getMode() == Mode.SIM) {
            simInstance = new SimPhotonVision(List.of(realCam),List.of(camera2Robot),field);
        }
        estimator = new PhotonPoseEstimator(field, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO, realCam, camera2Robot);
    }
    public static PhotonVisionSystem getInstance() {
        if (instance == null) {
            instance = new PhotonVisionSystem();
        }
        return instance;
    }
    public void refresh() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else {
            field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }
        if(Constants.getMode() == Mode.SIM) {
            simInstance.switchAllaince(field);
        }
        estimator = new PhotonPoseEstimator(field, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO, realCam, camera2Robot);
    }
    @Override
    public void periodic() {
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        Pose3d current3d = new Pose3d(currentPose);
        var results = realCam.getLatestResult();
        if (results.hasTargets()) {
            
            Optional<EstimatedRobotPose> poseEst = estimator.update();
            
            ArrayList<Pose3d> targets = new ArrayList<Pose3d>();
            ArrayList<Pose3d> estimates = new ArrayList<Pose3d>();
            for(PhotonTrackedTarget t :realCam.getLatestResult().getTargets()) {
                targets.add(current3d.transformBy(camera2Robot).transformBy(t.getBestCameraToTarget()));
                estimates.add(PhotonUtils.estimateFieldToRobotAprilTag(t.getBestCameraToTarget(), field.getTagPose(t.getFiducialId()).get() , camera2Robot.inverse()));
            }
            
            if(poseEst.isPresent()){
                Drivetrain.getInstance().addVisionPose(poseEst.get());
                Logger.recordOutput("photonvision/multitag", poseEst.get().estimatedPose);
            }
            Logger.recordOutput("photonvision/targetposes", targets.toArray(new Pose3d[targets.size()]));
            Logger.recordOutput("photonvision/poseEstimates", estimates.toArray(new Pose3d[estimates.size()]));
        }
        
        else {
            Logger.recordOutput("photonvision/targetposes", new Pose3d[] {});
            Logger.recordOutput("photonvision/poseEstimates", new Pose3d[] {});
        }
    }
    public static SimCameraProperties arducam_720() {
        var prop = new SimCameraProperties();
        prop.setCalibration(
                1280,
                720,
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill( // intrinsic
                        879.0720598169321,
                        0,
                        604.9212300278572,
                        0,
                        879.1507505285007,
                        389.16742755726875,
                                0.0,
                                0.0,
                                1.0),
                VecBuilder.fill( // distort
                -0.05910651726620547,
                0.04721812350044875,
                0.0023418721710541947,
                0.00018013724744152414,
                0.1608556267669461));
        prop.setCalibError(0.37, 0.06);
        prop.setFPS(30);
        prop.setAvgLatencyMs(40);
        prop.setLatencyStdDevMs(20);
        return prop;
    }

}
