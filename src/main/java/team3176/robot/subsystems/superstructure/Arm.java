package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Arm extends SubsystemBase {
    private static final double MAX_ENCODER_ANGLE_VALUE = SuperStructureConstants.ARM_HIGH_POS;
    private static final double MIN_ENCODER_ANGLE_VALUE = SuperStructureConstants.ARM_POOP_POS;
    private static final double MIN_COMMANDED_ANGLE = -32;
    private static Arm instance;
    private CANSparkMax armController;
    private CANCoder armEncoder;
    private double armEncoderAbsPosition;    
    private double lastEncoderPos;
    private final PIDController m_turningPIDController;


    private Arm() {
        armController = new CANSparkMax(Hardwaremap.arm_CID, MotorType.kBrushless);
        armEncoder = new CANCoder(Hardwaremap.armEncoder_CID);
        this.m_turningPIDController = new PIDController(SuperStructureConstants.ARM_kP, SuperStructureConstants.ARM_kI, SuperStructureConstants.ARM_kD);

        setArmPidPosMode();
    }

    private void setArmPidPosMode() {
        this.armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.armEncoder.configMagnetOffset(SuperStructureConstants.ARM_ENCODER_OFFSET);
        this.armEncoder.configSensorDirection(true,100);

        this.m_turningPIDController.setTolerance(SuperStructureConstants.ARM_TOLERANCE);
        //this.m_turningPIDController.enableContinuousInput() 
        this.m_turningPIDController.reset();
        this.m_turningPIDController.setP(SuperStructureConstants.ARM_kP);
        this.m_turningPIDController.setI(SuperStructureConstants.ARM_kI);
        this.m_turningPIDController.setD(SuperStructureConstants.ARM_kD);
        this.m_turningPIDController.enableContinuousInput(0, 360);
        this.armController.setOpenLoopRampRate(0.5);
        
        this.armController.burnFlash();
    }

    public static Arm getInstance() {
        if (instance == null){instance = new Arm();}
        return instance;
    }
    private double clampArmAngle(double angle) {
        if (angle > 200 && angle < Arm.MIN_ENCODER_ANGLE_VALUE){
            angle = Arm.MIN_ENCODER_ANGLE_VALUE;
        } else if (angle < 200 && angle > Arm.MAX_ENCODER_ANGLE_VALUE) {
            angle = Arm.MAX_ENCODER_ANGLE_VALUE;
        }
        return angle;
    }
    /**
     * 
     * @param desiredAngle in degrees in Encoder Frame
     */
    public void setPIDPosition(double desiredAngle) {
        //need to double check these values
        double physicsAngle = (desiredAngle - 200) ;
        this.armEncoderAbsPosition = armEncoder.getAbsolutePosition();
        
        //kg is the scalar representing the percent power needed to hold the arm at 90 degrees away from the robot
        double kg = SmartDashboard.getNumber("Arm_Kg", 0.1);
        // kp set as the fraction of control effort / error to cause control effort
        // for example .4 output is generated by a 40 degree error
        double kp = SmartDashboard.getNumber("Arm_kp", 0.4/40.0);
        m_turningPIDController.setP(kp);
        double feedForward = kg * Math.sin(desiredAngle);
        double turnOutput = m_turningPIDController.calculate(this.armEncoderAbsPosition, desiredAngle);
        turnOutput = MathUtil.clamp(turnOutput,-0.4,0.4);
        armController.set(turnOutput + feedForward);
    }

    public void armAnalogUp() {
        armController.set(SuperStructureConstants.ARM_OUTPUT_POWER);
    }
    public void armAnalogDown() {
        armController.set(-SuperStructureConstants.ARM_OUTPUT_POWER);
        System.out.println("Arm Analog Down"); 
    }
    public void idle() {
        armController.set(0.0);
    }
    
    /*
     * Commands
     */
    public Command armSetPosition(double angleInDegrees) {
        return this.run(() -> setPIDPosition(angleInDegrees));
    }
    public Command armAnalogUpCommand() {
        return this.runEnd(() -> armAnalogUp(), () -> idle());
    }
    public Command armAnalogDownCommand() {
        return this.runEnd(() -> armAnalogDown(), () -> idle());
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm_Position", armEncoder.getAbsolutePosition());
    }
}
