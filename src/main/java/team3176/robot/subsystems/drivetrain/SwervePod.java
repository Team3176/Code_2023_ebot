package team3176.robot.subsystems.drivetrain;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.DrivetrainHardwareMap;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.God.*;


public class SwervePod {

    /** Class Object holding the Motor controller for Drive Motor on the SwervePod */
    /** Current value in radians of the azimuthEncoder's position */
    double azimuthEncoderRelPosition;
    double azimuthEncoderAbsPosition;
    double desiredOptimizedAzimuthPosition;
    double velTicsPer100ms;
    boolean lastHasResetOccurred;
    
    /** Numerical identifier to differentiate between pods.
     *     For 4 Pods:  0 = FrontRight (FR),
     *                  1 = FrontLeft (FL),
     *                  2 = BackLeft (BL),
     *                  3 = BackRight (BR)
     */
    private int id;
    private String idString;
   
    /** Represents the value of the azimuthEncoder reading in radians when positioned with the positive Thrust vector of the Pod's Drive wheel pointing towards front of robot */
    //private double kAzimuthEncoderUnitsPerRevolution;

    private double lastEncoderPos;

    //private double kP_Azimuth;
    private LoggedTunableNumber kPAzimuth = new LoggedTunableNumber("kP_azimuth");
    private LoggedTunableNumber kIAzimuth = new LoggedTunableNumber("kI_azimuth");
    private double kDAzimuth;
    private double lastDistance =0.0;
    private double delta = 0.0;
    private LoggedTunableNumber velMax = new LoggedTunableNumber("az_vel");
    private LoggedTunableNumber velAcc = new LoggedTunableNumber("az_acc");

    private final PIDController  turningPIDController;
    //private final ProfiledPIDController m_turningProfiledPIDController;
    //private ProfiledPIDController m_turningPIDController;

    private SwervePodIO io;
    private SwervePodIOInputsAutoLogged inputs = new SwervePodIOInputsAutoLogged();

    public SwervePod(int id, SwervePodIO io) {
        this.id = id;
        this.io = io;
        this.desiredOptimizedAzimuthPosition = 0.0;

        //this.kP_Azimuth = 0.006;
        kPAzimuth.initDefault(.007);
        this.kIAzimuth.initDefault(0.0);
        this.kDAzimuth = 0.0;
        velMax.initDefault(900);
        velAcc.initDefault(900);

        turningPIDController = new PIDController(kPAzimuth.get(), kIAzimuth.get(), kDAzimuth);//,new Constraints(velMax.get(), velAcc.get()));
        turningPIDController.setTolerance(4);
        turningPIDController.enableContinuousInput(-180, 180);
        turningPIDController.setIntegratorRange(-0.1,0.1);
        turningPIDController.setP(this.kPAzimuth.get());
        turningPIDController.setI(this.kIAzimuth.get());
        turningPIDController.setD(this.kDAzimuth);
        
    }


    public void setModule(double speedMetersPerSecond, Rotation2d angle) {
        setModule(new SwerveModuleState(speedMetersPerSecond,angle));
    }

    /**
     *  alternative method for setting swervepod in line with WPILIB standard library
     * @param desiredState 
     */
    public SwerveModuleState setModule(SwerveModuleState desiredState) {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + Integer.toString(this.id), inputs);
        
        this.azimuthEncoderAbsPosition = inputs.turnAbsolutePositionDegrees;
        SwerveModuleState desiredOptimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(this.azimuthEncoderAbsPosition));
        this.desiredOptimizedAzimuthPosition = desiredOptimized.angle.getDegrees();
        double turnOutput;
        if (desiredState.speedMetersPerSecond > (-Math.pow(10,-10)) && desiredState.speedMetersPerSecond  < (Math.pow(10,-10))) {      
            turnOutput = turningPIDController.calculate(this.azimuthEncoderAbsPosition, this.lastEncoderPos);
        } else {
            turnOutput = turningPIDController.calculate(this.azimuthEncoderAbsPosition, desiredOptimized.angle.getDegrees());
            this.lastEncoderPos = desiredOptimized.angle.getDegrees(); 
        }
        // reduce output if the error is high
        double currentDistance = Units.feetToMeters((DrivetrainConstants.WHEEL_DIAMETER_INCHES/12.0 * Math.PI)  *  inputs.drivePositionRad / (2*Math.PI));
        this.delta = currentDistance - this.lastDistance;
        this.lastDistance = currentDistance;
        
        desiredOptimized.speedMetersPerSecond *= Math.abs(Math.cos(desiredOptimized.angle.minus(Rotation2d.fromDegrees(azimuthEncoderAbsPosition)).getRadians()));
        //Logger.getInstance().recordOutput("Drive/Module" + Integer.toString(this.id) + "", id);
        
        io.setTurn(MathUtil.clamp(turnOutput, -0.4, 0.4));
        Logger.getInstance().recordOutput("Drive/Module" + Integer.toString(this.id) + "/error",turningPIDController.getPositionError());
        //Logger.getInstance().recordOutput("Drive/Module" + Integer.toString(this.id) + "/setpoint",turningPIDController.getSetpoint().position);
        this.velTicsPer100ms = Units3176.mps2ums(desiredOptimized.speedMetersPerSecond);
        io.setDrive(desiredOptimized.speedMetersPerSecond);

        if(kPAzimuth.hasChanged(hashCode()) || kIAzimuth.hasChanged(hashCode())) {
            turningPIDController.setP(kPAzimuth.get());
            turningPIDController.setI(kIAzimuth.get());
        }
        // if(velAcc.hasChanged(hashCode()) || velMax.hasChanged(hashCode())){
        //     turningPIDController.setConstraints(new Constraints(velMax.get(),velAcc.get()));
        // }

        return desiredOptimized;
    }   
    /*
     * odometry calls
     */
    public SwerveModulePosition getPosition() {
        double mps = Units.feetToMeters((DrivetrainConstants.WHEEL_DIAMETER_INCHES/12.0 * Math.PI)  *  inputs.drivePositionRad / (2*Math.PI));
        return new SwerveModulePosition(mps,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
    }
    public SwerveModulePosition getDelta() {
        return new SwerveModulePosition(this.delta,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
    }
    
    public double getVelocity() {
        double wheelVelocityInFeetPerSecond = inputs.driveVelocityRadPerSec / (Math.PI *2) * DrivetrainConstants.WHEEL_DIAMETER_INCHES/12.0 * Math.PI;   
        return Units3176.feetPerSecond2metersPerSecond(wheelVelocityInFeetPerSecond);
    
    }

    /**
     * Returns current Azimuth of pod in degrees, where 0 is straight forward.
     * @return
     */
    public double getAzimuth() {
        return inputs.turnAbsolutePositionDegrees; 
    }

    public void setThrustCoast() {
        io.setDriveBrakeMode(false);
    }

    public void setThrustBrake() {
        io.setDriveBrakeMode(true);
    }

    public double getAzimuthSetpoint() {
        return this.desiredOptimizedAzimuthPosition;
    }
    public double getThrustSetpoint() {
        return this.velTicsPer100ms;
    }
    public double getThrustEncoderVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    public void setupShuffleboard() {
        Shuffleboard.getTab(this.idString)
            .add(idString+"/podAzimuth_setpoint_angle",DrivetrainHardwareMap.AZIMUTH_ABS_ENCODER_OFFSET_POSITION[id])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -3.16, "max", 3.16))
            .withSize(2,1)
            .withPosition(2,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kP_Azimuth", this.kPAzimuth.get())
            .withSize(1,1)
            .withPosition(4,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kI_Azimuth", this.kIAzimuth)
            .withSize(1,1)
            .withPosition(5,1)
            .getEntry();
        Shuffleboard.getTab(this.idString)
            .add(idString+"/kD_Azimuth", this.kDAzimuth)
            .withSize(1,1)
            .withPosition(6,1)
            .getEntry();
    }
}
