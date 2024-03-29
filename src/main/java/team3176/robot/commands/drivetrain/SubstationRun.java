package team3176.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.vision.Vision;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.DoubleSubscriber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SubstationRun {
    private Drivetrain m_Drivetrain;
    public NetworkTableInstance tableInstance;
    public NetworkTable ll_lpov, ll_rpov;
    public double ltv, rtv, tv, ltx, rtx, tx, lty, rty, ty, lta, rta, ta; 
    public DoubleTopic aprilIDTopic;
    public DoubleSubscriber aprilID;
    public double wantedAprilID, wantedYaw, wantedXpos, wantedYpos;
    public Alliance alliance;
    public PathPlannerTrajectory traja;
    public PPSwerveControllerCommand swerveCommand;
    private String side;
    
    public SubstationRun(String side){
        m_Drivetrain = Drivetrain.getInstance();
        this.side = side;

    }
    
    public void initialize(){
        //m_Drivetrain.setBrakeMode();
        tableInstance = NetworkTableInstance.getDefault();
        ll_lpov = tableInstance.getTable("limelight-lpov");
        ll_rpov = tableInstance.getTable("limelight-rpov");

        if (DriverStation.isFMSAttached() && (alliance == null)) {
            alliance = DriverStation.getAlliance();
            if (alliance == Alliance.Red){
                wantedAprilID = 5.0;
            } else if (alliance == Alliance.Blue){
                wantedAprilID = 4.0;
            } else if (alliance == Alliance.Invalid){
                wantedAprilID = 9.0;
            }
        }

        if (wantedAprilID == 5.0) {
            if (this.side == "left") {
                wantedXpos = 68; //placeholder until we get the bot
                wantedYpos = 680; //placeholder until we get the bot
                wantedYaw = 0;//placeholder until we get the bot
            } else if (this.side == "right") {
                wantedXpos = 68; //placeholder until we get the bot
                wantedYpos = 68; //placeholder until we get the bot
                wantedYaw = 0;//placeholder until we get the bot
            }
        } else if (wantedAprilID == 4.0) {
            if (this.side == "left") {
                wantedXpos = 68; //placeholder until we get the bot
                wantedYpos = 68; //placeholder until we get the bot
                wantedYaw = 0;//placeholder until we get the bot
            } else if (this.side == "right") {
                wantedXpos = 68; //placeholder until we get the bot
                wantedYpos = 68; //placeholder until we get the bot
                wantedYaw = 0;//placeholder until we get the bot
            }
        }

        ltv = ll_lpov.getEntry("tv").getDouble(0); 
        rtv = ll_rpov.getEntry("tv").getDouble(0); 

        if (ltv == 1) { 
            
        }



    }
    public void execute(){
        Pose2d pose = m_Drivetrain.getPose();
        double xpos = pose.getX();
        double ypos = pose.getY();
   
        //aprilIDTopic = limelightTable.getDoubleTopic("tid");
        aprilID = aprilIDTopic.subscribe(0.0);
        double forward = 0.0;
        if (aprilID.getAsDouble() == wantedAprilID ){

            traja = PathPlanner.generatePath(
                new PathConstraints(1, 1), 
                new PathPoint(new Translation2d(wantedXpos, wantedYpos),new Rotation2d(0), pose.getRotation()), // position, heading
                new PathPoint(new Translation2d(wantedXpos,wantedYpos),new Rotation2d(0), new Rotation2d(wantedYaw)) // position, heading
            );
            swerveCommand = new PPSwerveControllerCommand(traja, m_Drivetrain::getPose, DrivetrainConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
                new PIDController(5.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(5.0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                m_Drivetrain::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                m_Drivetrain);
            swerveCommand.initialize();
        }
    }
}

