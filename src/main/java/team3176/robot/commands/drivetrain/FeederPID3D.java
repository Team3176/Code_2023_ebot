package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;


import team3176.robot.subsystems.drivetrain.Drivetrain;

public class FeederPID3D extends CommandBase{
    Drivetrain drivetrain;
    PIDController xController = new PIDController(2.0,0.0,0.0);
    PIDController yController = new PIDController(2.0,0.0,0.0);
    Pose2d redRight = new Pose2d(1.17, 7.44, Rotation2d.fromDegrees(180));
    Pose2d redLeft = new Pose2d(1.17, 6.17, Rotation2d.fromDegrees(180));
    Pose2d blueRight = new Pose2d(15.44, 6.17, Rotation2d.fromDegrees(0.0));
    Pose2d blueLeft = new Pose2d(15.44, 7.44, Rotation2d.fromDegrees(0.0));
    Pose2d targetPose;
    NetworkTable vision;
    Alliance alliance;
    String side;
    public FeederPID3D(String side) {
        this.side = side;
        alliance = DriverStation.getAlliance();
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
        vision = NetworkTableInstance.getDefault().getTable("limelight");
        if(side.equals("right")) {
            if(DriverStation.getAlliance() == Alliance.Red) {
                targetPose = redRight;
            } else {
                targetPose = blueRight;
            }
        } else {
            if(DriverStation.getAlliance() == Alliance.Red) {
                targetPose = redLeft;
            } else {
                targetPose = blueLeft;
            }
        }
    }
    @Override
    public void initialize(){
        drivetrain.setSpinLock(true);
        drivetrain.setSpinLockAngle(targetPose.getRotation().getDegrees()); 
    }
    @Override
    public void execute() {
        double[] defaultPose = {0.0,0.0,0.0,0.0,0.0,0.0};
        double[] visionPoseArray = vision.getEntry("botpose_wpiblue").getDoubleArray(defaultPose);
        Pose2d camPose = new Pose2d(visionPoseArray[0],visionPoseArray[1],Rotation2d.fromDegrees(visionPoseArray[5]));
        double tv = vision.getEntry("tv").getDouble(0.0);
        double reverseAxis = DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0;
        if (tv != 0.0) {
            drivetrain.drive(MathUtil.clamp(reverseAxis*xController.calculate(camPose.getX(), targetPose.getX()),-1.5,1.5),
                            (MathUtil.clamp(reverseAxis*yController.calculate(camPose.getY(),targetPose.getY()),-1.5,1.5)),
                            0.0);
        } else drivetrain.drive (Math.pow(10,-7),Math.pow(10,-7),Math.pow(10,-7));
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpinLock(false);
    }
}
