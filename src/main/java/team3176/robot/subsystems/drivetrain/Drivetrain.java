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

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

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


  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;

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
  Field2d field;
  Pose3d visionPose3d;
  SimNoNoiseOdom simNoNoiseOdom;
  
  // private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  private Drivetrain(GyroIO io) {
    this.io = io;
    inputs = new GyroIOInputs();

    field = new Field2d();
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


    this.forwardCommand = Math.pow(10, -15); // Has to be positive to turn that direction?
    this.strafeCommand = 0.0;
    this.spinCommand = 0.0;
    vision = NetworkTableInstance.getDefault().getTable("limelight");

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

  /**
   * public facing drive command that allows command to specify if the command is
   * field centric or not
   * 
   * @param forwardCommand meters per second
   * @param strafeCommand  meters per second
   * @param spinCommand    meters per second
   * @param type           FIELD CENTRIC or ROBOT_CENTRIC
   */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand, coordType type) {
    
    this.currentCoordType = type;
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    
  }

  /**
   * default call will assume robot_centric
   * 
   * @param forwardCommand
   * @param strafeCommand
   * @param spinCommand
   */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand) {
    drive(forwardCommand, strafeCommand, spinCommand, currentCoordType);
  }

  private ChassisSpeeds getCurrentChassisSpeedRequest() {
    ChassisSpeeds currChassisSpeeds = new ChassisSpeeds(forwardCommand, strafeCommand, spinCommand);
    if (this.currentCoordType == coordType.FIELD_CENTRIC) {
      Rotation2d fieldOffset = this.getPose().getRotation();
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        fieldOffset.plus(Rotation2d.fromDegrees(180));
      }
      currChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(currChassisSpeeds, fieldOffset);
    }
    if (isSpinLocked) {
      currChassisSpeeds.omegaRadiansPerSecond = spinLockPID.calculate(getPoseYawWrapped().getDegrees(), spinLockAngle.getDegrees());
      SmartDashboard.putNumber("SpinLockYaw",getPoseYawWrapped().getDegrees());
    }
    return currChassisSpeeds;
  }
  /**
   * Robot Centric Forward, strafe, and spin to set individual pods commanded spin
   * speed and drive speed
   * 
   * @param forwardCommand meters per second
   * @param strafeCommand  meters per second
   * @param spinCommand    meters per second
   */
  private void calculateNSetPodPositions() {
    ChassisSpeeds currChassisSpeeds = getCurrentChassisSpeedRequest();
    SwerveModuleState[] podStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(currChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(podStates, DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int idx = 0; idx < (pods.size()); idx++) {
      optimizedStates[idx]=pods.get(idx).setModule(podStates[idx]);
    }
    Logger.recordOutput("SwerveStates/Setpoints", podStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    Logger.recordOutput("Drive/SpinCommand", spinCommand);
  }

  private void recordRealPods() {
    SwerveModuleState[] realStates = new SwerveModuleState[4];
    for (int idx = 0; idx < (pods.size()); idx++) {
      realStates[idx] = new SwerveModuleState(pods.get(idx).getVelocity(),Rotation2d.fromDegrees(pods.get(idx).getAzimuth()));
    }
    Logger.recordOutput("SwerveStates/real", realStates);
  }


  public void setDriveMode(driveMode wantedDriveMode) {
    this.currentDriveMode = wantedDriveMode;
  }

  public driveMode getCurrentDriveMode() {
    return this.currentDriveMode;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
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
  public double getCurrentChassisSpeed(){
    return Math.sqrt(Math.pow(this.forwardCommand,2) +  Math.pow(this.strafeCommand,2));
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
    return this.runOnce(() -> this.setDriveMode(driveMode.DRIVE)).andThen(this.run(() -> drive(forward.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND,strafe.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND,spin.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND)));
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/gyro", inputs);
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
    
    
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/Odom", getPoseOdom());
    
    SmartDashboard.putNumber("NavYaw",getPoseYawWrapped().getDegrees());

    //set pods
    recordRealPods();
    switch(currentDriveMode) {
      case CUBECHASEAUTON:
        this.spinCommand = calcCubeChaseSpinCommand();
        calculateNSetPodPositions();
        break;
      case CUBECHASETELEOP:
        if (LimelightHelpers.getTV("limelight-three")) {
          this.spinCommand = calcCubeChaseSpinCommand();
        }
        this.forwardCommand = -1 * this.forwardCommand;
        //this.strafeCommand = -1 * this.strafeCommand;
        this.strafeCommand = 0.0;
        calculateNSetPodPositions();
        break;
      case DEFENSE:
        pods.get(0).setModulePositionOnly(Rotation2d.fromDegrees(-45));
        pods.get(1).setModulePositionOnly(Rotation2d.fromDegrees(45));
        pods.get(2).setModulePositionOnly(Rotation2d.fromDegrees(-45));
        pods.get(3).setModulePositionOnly(Rotation2d.fromDegrees(45));
        break;
      case DRIVE:
        calculateNSetPodPositions();
        break;
      case AUTO:
        break;
      default:
        calculateNSetPodPositions();
        break;
    }
    
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);

    //publishSwervePodPIDErrors();
    // SmartDashboard.putBoolean("Defense", currentDriveMode == driveMode.DEFENSE);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void publishSwervePodPIDErrors(){
    final double FRAzError = podFR.getAzimuthSetpoint() - podFR.getAzimuth();
    final double FRThrustError = podFR.getThrustSetpoint() - podFR.getThrustEncoderVelocity();
    SmartDashboard.putNumber("FRAzError", FRAzError);
    SmartDashboard.putNumber("FRThrustError", FRThrustError);

    final double FLAzError = podFL.getAzimuthSetpoint() - podFL.getAzimuth();
    final double FLThrustError = podFL.getThrustSetpoint() - podFL.getThrustEncoderVelocity();
    SmartDashboard.putNumber("FLAzError", FLAzError);
    SmartDashboard.putNumber("FLThrustError", FLThrustError);

    final double BRAzError = podBR.getAzimuthSetpoint() - podBR.getAzimuth();
    final double BRThrustError = podBR.getThrustSetpoint() - podBR.getThrustEncoderVelocity();
    SmartDashboard.putNumber("BRAzError", BRAzError);
    SmartDashboard.putNumber("BRThrustError", BRThrustError);

    final double BLAzError = podBL.getAzimuthSetpoint() - podBL.getAzimuth();
    final double BLThrustError = podBL.getThrustSetpoint() - podBL.getThrustEncoderVelocity();
    SmartDashboard.putNumber("BLAzError", BLAzError);
    SmartDashboard.putNumber("BLThrustError", BLThrustError);

  }
  
  
}
