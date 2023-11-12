// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.DrivetrainHardwareMap;
import team3176.robot.constants.SwervePodHardwareID;
import team3176.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;
import team3176.robot.util.LocalADStarAK;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private SwerveDriveOdometry odom;
  private SwerveDrivePoseEstimator poseEstimator;

  // private Controller controller = Controller.getInstance();
  // private Vision m_Vision = Vision.getInstance();
  public enum coordType {
    FIELD_CENTRIC, ROBOT_CENTRIC
  }

  private coordType currentCoordType = coordType.FIELD_CENTRIC;
  //private PowerDistribution PDH = new PowerDistribution();
  // PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);

  private ArrayList<SwervePod> pods;

  private driveMode currentDriveMode = driveMode.DRIVE;

  

  Rotation2d fieldAngleOffset = Rotation2d.fromDegrees(0.0);


  // spin lock
  private PIDController spinLockPID;

  private Rotation2d spinLockAngle = Rotation2d.fromDegrees(0.0);
  private boolean isSpinLocked = false;

  private boolean isTurboOn = false;
  PIDController CubeChaseController = new PIDController(0.08, 0, 0);
  double cubeChaseSpinCommand;
  double cubeTx;
  //private int arraytrack;
  double[] angleHist = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  double angleAvgRollingWindow;

  public enum driveMode {
    DEFENSE, DRIVE, CUBECHASETELEOP, CUBECHASEAUTON, AUTO
  }

  private SwervePod podFR;
  private SwervePod podFL;
  private SwervePod podBL;
  private SwervePod podBR;


  NetworkTable vision;
  NetworkTableEntry visionPose;
  Pose2d lastPose = new Pose2d();
  double lastVisionTimeStamp = 0.0;
  double lastVisionX = 0.0;
  Rotation2d wheelOnlyHeading = new Rotation2d();
  private final GyroIO io;
  private GyroIOInputs inputs;
  Pose3d visionPose3d;
  SimNoNoiseOdom simNoNoiseOdom;
  
  // private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  private Drivetrain(GyroIO io) {
    this.io = io;
    inputs = new GyroIOInputs();

    // check for duplicates
    assert (!SwervePodHardwareID.check_duplicates_all(DrivetrainHardwareMap.FR, DrivetrainHardwareMap.FL,
        DrivetrainHardwareMap.BR, DrivetrainHardwareMap.BL));
    // Instantiate pods
    if(Constants.getMode() != Mode.REPLAY) {
      switch(Constants.getRobot()){
        case ROBOT_2023C:
          System.out.println("[init] normal swervePods");
          DrivetrainHardwareMap.FR.OFFSET += 180;
          DrivetrainHardwareMap.FL.OFFSET += 90;
          DrivetrainHardwareMap.BL.OFFSET += 0;
          DrivetrainHardwareMap.BR.OFFSET += -90;
          podFR = new SwervePod(0, new SwervePodIOFalconSpark(DrivetrainHardwareMap.FR,DrivetrainHardwareMap.STEER_FR_CID));
          podFL = new SwervePod(1, new SwervePodIOFalconSpark(DrivetrainHardwareMap.FL,DrivetrainHardwareMap.STEER_FL_CID));
          podBL = new SwervePod(2, new SwervePodIOFalconSpark(DrivetrainHardwareMap.BL,DrivetrainHardwareMap.STEER_BL_CID));
          podBR = new SwervePod(3, new SwervePodIOFalconSpark(DrivetrainHardwareMap.BR,DrivetrainHardwareMap.STEER_BR_CID));
          break;
        case ROBOT_2023P:
          break;
        case ROBOT_SIMBOT:
          System.out.println("[init] simulated swervePods");
          podFR = new SwervePod(0, new SwervePodIOSim());
          podFL = new SwervePod(1, new SwervePodIOSim());
          podBL = new SwervePod(2, new SwervePodIOSim());
          podBR = new SwervePod(3, new SwervePodIOSim());
          simNoNoiseOdom = new SimNoNoiseOdom(new ArrayList<>(List.of(podFR, podFL, podBL, podBR)));
          break;
        default:
          break;
        
      }
    } else {
      podFR = new SwervePod(0, new SwervePodIO(){});
      podFL = new SwervePod(1, new SwervePodIO(){});
      podBL = new SwervePod(2, new SwervePodIO(){});
      podBR = new SwervePod(3, new SwervePodIO(){});
    }

    // Instantiate array list then add instantiated pods to list
    pods = new ArrayList<>();
    pods.add(podFR);
    pods.add(podFL);
    pods.add(podBL);
    pods.add(podBR);

    
    visionPose3d = new Pose3d();
    odom = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS, this.getSensorYaw(),
        new SwerveModulePosition[] {
            podFR.getPosition(),
            podFL.getPosition(),
            podBL.getPosition(),
            podBR.getPosition()
        }, new Pose2d(0.0, 0.0, new Rotation2d()));
    poseEstimator = new SwerveDrivePoseEstimator(DrivetrainConstants.DRIVE_KINEMATICS, getSensorYaw(),
        getSwerveModulePositions(), odom.getPoseMeters());
    spinLockPID = new PIDController(0.1, 0.0, 0.0);
    // set for max and min of degrees for Rotation2D
    spinLockPID.enableContinuousInput(-180, 180);

    //arraytrack = 0;
    angleAvgRollingWindow = 0;


    vision = NetworkTableInstance.getDefault().getTable("limelight");
    
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        () -> DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
        this::driveVelocity,
        new HolonomicPathFollowerConfig(
            4.0, DrivetrainConstants.EBOT_LENGTH_IN_METERS_2023, new ReplanningConfig()),
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() {
    if (instance == null) {
      if(Constants.getMode() != Mode.SIM) {
        instance = new Drivetrain(new GyroIONavX());
      }
      else{
        instance = new Drivetrain(new GyroIO() {});
      }
    }
    return instance;
  }


  public void driveVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] podStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(podStates, DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int idx = 0; idx < (pods.size()); idx++) {
      optimizedStates[idx]=pods.get(idx).setModule(podStates[idx]);
    }
    Logger.recordOutput("Drivetrain/Setpoints", podStates);
    Logger.recordOutput("Drivetrain/SetpointsOptimized", optimizedStates);
  }

  public void driveVelocityFieldCentric(ChassisSpeeds speeds) {
    Rotation2d fieldOffset = this.getPose().getRotation();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      fieldOffset.plus(Rotation2d.fromDegrees(180));
    }
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, fieldOffset);
    driveVelocity(speeds);
  }

  @AutoLogOutput
  private SwerveModuleState[] getSwerveStatesReal() {
    SwerveModuleState[] realStates = new SwerveModuleState[4];
    for (int idx = 0; idx < (pods.size()); idx++) {
      realStates[idx] = new SwerveModuleState(pods.get(idx).getVelocity(),Rotation2d.fromDegrees(pods.get(idx).getAzimuth()));
    }
    return realStates;
  }


  public void setDriveMode(driveMode wantedDriveMode) {
    this.currentDriveMode = wantedDriveMode;
  }

  public driveMode getCurrentDriveMode() {
    return this.currentDriveMode;
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  @AutoLogOutput
  public Pose2d getPoseOdom() {
    return odom.getPoseMeters();
  }
  public Pose2d getSimNoNoisePose() {
    return simNoNoiseOdom.getPoseTrue();
  }
  public void addVisionPose(EstimatedRobotPose p) {
    Matrix<N3,N1> cov = new Matrix<>(Nat.N3(), Nat.N1());
    double distance = 0.0;
    for(var t : p.targetsUsed) {
      distance += t.getBestCameraToTarget().getTranslation().getNorm() / p.targetsUsed.size();
    }
    Logger.recordOutput("Drivetrain/distance2target", distance);
    if (p.targetsUsed.size() > 1) {
      //multi tag
      double distance2 = Math.max(Math.pow(distance * 0.2,2),0.7);
      cov = VecBuilder.fill(distance2,distance2,0.9);
    } else {
      double distance2 = Math.pow(distance * 0.5, 2);
      cov = VecBuilder.fill(distance2,distance2,distance2);
    }
    poseEstimator.addVisionMeasurement(p.estimatedPose.toPose2d(), p.timestampSeconds,cov);
  }

  public void resetPose(Pose2d pose) {
    wheelOnlyHeading = pose.getRotation();
    odom.resetPosition(getSensorYaw(), new SwerveModulePosition[] {
        podFR.getPosition(),
        podFL.getPosition(),
        podBL.getPosition(),
        podBR.getPosition() }, pose);
    poseEstimator.resetPosition(getSensorYaw(), new SwerveModulePosition[] {
          podFR.getPosition(),
          podFL.getPosition(),
          podBL.getPosition(),
          podBR.getPosition() }, pose);
    if(Constants.getMode() == Mode.SIM) {
      simNoNoiseOdom.resetSimPose(pose);
    }
    
  }
  public void resetPoseToVision() {
    this.resetPose(visionPose3d.toPose2d());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    currentDriveMode = driveMode.AUTO;
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setModule(states[idx]);
    }
  }
  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = pods.get(i).getState();
    }
    return states;
  }

  /**
   * Sets Turbo mode on or off
   * 
   * @param onOrOff Passing a value of true sets Turbo on (ie isTurboOn = true),
   *                and passing value of false sets Turbo off (ie isTurboOn =
   *                false)
   */
  public void setTurbo(boolean onOrOff) {
    this.isTurboOn = onOrOff;
  }

  public void setCoordType(coordType c) {
    this.currentCoordType = c;
  }

  public void setSpinLock(boolean b) {
    this.isSpinLocked = b;
  }

  public void setSpinLockAngle() {
    this.spinLockAngle = getSensorYaw();
  }

  public void setSpinLockAngle(double angle) {
    this.spinLockAngle = Rotation2d. fromDegrees(angle);
  }
  /**
   * 
   * @return returns the chassis yaw wrapped between -pi and pi
   */
  public Rotation2d getPoseYawWrapped() {
    // its ugly but rotation2d is continuos but I imagine most of our applications
    // we want it bounded between -pi and pi
    return Rotation2d
        .fromRadians(MathUtil.angleModulus(getPose().getRotation().getRadians()));
  }

  /**
   * The unbounded angle
   * 
   * @return Rotation2d of the yaw
   */
  protected Rotation2d getSensorYaw() {
    if(Constants.getMode() == Mode.SIM) {
      if(this.odom == null || this.poseEstimator == null) {
        return new Rotation2d();
      }
      return simNoNoiseOdom.getPoseTrue().getRotation();
    } 
    return inputs.rotation2d;
    
    
  }

  public Rotation2d getChassisYaw() {
    return getPose().getRotation();
  }

  /**
   * 
   * @return navx pitch -180 to 180 around the X axis of the Navx
   */
  public double getChassisPitch() {
    return inputs.pitch;
  }

  /**
   * 
   * @return navx roll -180 to 180 around the X axis of the Navx
   */
  public double getChassisRoll() {
    return inputs.roll;
  }


  public void resetFieldOrientation() {
    // do not need to invert because the navx rotation2D call returns a NWU
    // coordsys!
    //this.FieldAngleOffset = m_NavX.getRotation2d();
    Rotation2d redOrBlueZero = new Rotation2d();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      redOrBlueZero.plus(Rotation2d.fromDegrees(180));
    }
    resetPose(new Pose2d(getPose().getTranslation(),redOrBlueZero));
  }

  public double getPodVelocity(int podID) {
    return pods.get(podID).getVelocity();
  }

  public double getPodAzimuth(int podID) {
    return pods.get(podID).getAzimuth();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        podFR.getPosition(),
        podFL.getPosition(),
        podBL.getPosition(),
        podBR.getPosition()
    };
  }
  

  public void setCoastMode() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setThrustCoast();
    }
  }

  public void setBrakeMode() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setThrustBrake();
    }
  }
  /*
   * public ChassisSpeeds getChassisSpeed() {
   * return DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(podFR.getState(),
   * podFL.getState(), podBL.getState(), podBR.getState());
   * }
   */


  private double calcCubeChaseSpinCommand() {  
      this.cubeTx = LimelightHelpers.getTX("limelight-three");
      cubeChaseSpinCommand = 1 * CubeChaseController.calculate(cubeTx, 0.0);
      return cubeChaseSpinCommand;
  }

/* 
  public Command setCubeChaseOn() {
    return new InstantCommand(() -> setDriveMode(driveMode.CUBECHASE)); 
  }
  
  public Command setCubeChaseOff() {
    return new InstantCommand(() -> setDriveMode(driveMode.DRIVE)); 
  }
*/
  public Command setFieldCentric() {
     // using instant command because I do not want these to interupt other drivetrain commands
    return new InstantCommand(() -> this.setCoordType(coordType.FIELD_CENTRIC)).withName("setFieldCentric");
  }
  public Command setRobotCentric() {
    // using instant command because I do not want these to interupt other drivetrain commands
    return new InstantCommand(() -> this.setCoordType(coordType.ROBOT_CENTRIC)).withName("setRobotCentric");
  }
  public Command swerveDefenseCommand() {
    return this.runEnd(() -> this.setDriveMode(driveMode.DEFENSE), () -> this.setDriveMode(driveMode.DRIVE));
  }
  public Command swerveDrivePercent(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier spin) {
    return this.runOnce(() -> this.setDriveMode(driveMode.DRIVE)).andThen(this.run(() -> driveVelocityFieldCentric( new ChassisSpeeds(forward.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND,strafe.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND,spin.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND))));
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drivetrain/gyro", inputs);
    lastPose = poseEstimator.getEstimatedPosition();
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];
    
    for(int i=0;i<  pods.size(); i++) {
        deltas[i] = pods.get(i).getDelta();
    }
    Twist2d twist = DrivetrainConstants.DRIVE_KINEMATICS.toTwist2d(deltas);
    wheelOnlyHeading = getPoseOdom().exp(twist).getRotation();
    // update encoders
    this.poseEstimator.update(getSensorYaw(), getSwerveModulePositions());
    this.odom.update(getSensorYaw(), getSwerveModulePositions());
    if(Constants.getMode() == Mode.SIM) {
      simNoNoiseOdom.update();
    }
    
    
    SmartDashboard.putNumber("NavYaw",getPoseYawWrapped().getDegrees());

    //set pods
    switch(currentDriveMode) {
      case CUBECHASEAUTON:
       //
        break;
      case CUBECHASETELEOP:
        // if (LimelightHelpers.getTV("limelight-three")) {
        //   this.spinCommand = calcCubeChaseSpinCommand();
        // }
        // this.forwardCommand = -1 * this.forwardCommand;
        // //this.strafeCommand = -1 * this.strafeCommand;
        // this.strafeCommand = 0.0;
        break;
      case DEFENSE:
        DrivetrainConstants.DRIVE_KINEMATICS.resetHeadings(Rotation2d.fromDegrees(-45),
                                                            Rotation2d.fromDegrees(45),
                                                            Rotation2d.fromDegrees(-45),
                                                            Rotation2d.fromDegrees(45));
        this.driveVelocity(new ChassisSpeeds());
        break;
    }
    

    //publishSwervePodPIDErrors();
    // SmartDashboard.putBoolean("Defense", currentDriveMode == driveMode.DEFENSE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  
}
