package team3176.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
public class DrivetrainConstants extends DrivetrainHardwareMap {
    // IDs for Drivetrain motors and solenoids
  

    // Drivetrain dimensions for kinematics and odometry
    public static final double EBOT_LENGTH_IN_METERS_2023 = Units.inchesToMeters(24.3); 
    public static final double EBOT_WIDTH_IN_METERS_2023 = Units.inchesToMeters(28.75);
    public static final double LENGTH = EBOT_LENGTH_IN_METERS_2023;
    public static final double WIDTH = EBOT_WIDTH_IN_METERS_2023;

    public static final double WHEEL_DIAMETER_INCHES = 3.00; // Inches
    public static final double MAX_WHEEL_SPEED_METERS_PER_SECOND = Units.feetToMeters(14.0);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(LENGTH / 2.0, -WIDTH / 2.0),  //FR where +x=forward and -y=starboard
        new Translation2d(LENGTH / 2.0, WIDTH / 2.0),   //FL where +x=forward and +y=port
        new Translation2d(-LENGTH / 2.0, WIDTH / 2.0), //BL where -x=backward(aft) and +y=port
        new Translation2d(-LENGTH / 2.0, -WIDTH/ 2.0)   //BR where -x=backward(aft) and -y=starboard
    );

    //SwervePod Constants
    private static final double AZIMUTH_GEAR_RATIO = 70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
    public static final double THRUST_GEAR_RATIO = (14.0/22.0) * (15.0/45.0);  

    public static final double AZIMUTH_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final double THRUST_ENCODER_UNITS_PER_REVOLUTION = 2048;

    public static final double[] AUTON_THETA_CONTROLLER_PIDF = { 3.0 /*kP*/, 0.0 /*kI*/, 0.0 /*kD*/, 0.0 /*kF*/};

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            //MAX_ROT_SPEED_RADIANS_PER_SECOND, MAX_ROT_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
            2*Math.PI, 2*Math.PI
        );
    
}
