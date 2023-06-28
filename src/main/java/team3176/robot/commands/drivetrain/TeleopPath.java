package team3176.robot.commands.drivetrain;

import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class TeleopPath extends CommandBase{
    Drivetrain drivetrain;
    PathPlannerTrajectory traj1;
    PPSwerveControllerCommand swerveCommand;
    public TeleopPath() {
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }
    @Override
    public void initialize(){
        Pose2d pose = drivetrain.getPose();
        double xposition = pose.getX();
        double yposition = pose.getY();
        //System.out.println("pose" + xposition + "," + yposition);
        traj1 = PathPlanner.generatePath(
          new PathConstraints(1, 1), 
          new PathPoint(new Translation2d(xposition, yposition), pose.getRotation(), pose.getRotation(), drivetrain.getCurrentChassisSpeed()), // position, heading
          new PathPoint(new Translation2d( 1.6, 6.74),Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180),.01), // position, heading
          new PathPoint(new Translation2d( 1.1, 6.74),Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180))
        );
        //System.out.println("traj" + traj1.getTotalTimeSeconds());
        swerveCommand = new PPSwerveControllerCommand(traj1, drivetrain::getPose, DrivetrainConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
        new PIDController(5.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(5.0, 0, 0), // Y controller (usually the same values as X controller)
        new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        drivetrain::setModuleStates, // Module states consumer
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drivetrain);
        swerveCommand.initialize();
    }
    @Override
    public void execute() {
        swerveCommand.execute();
    }
    @Override
    public boolean isFinished() {
        return swerveCommand.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        swerveCommand.end(interrupted);
    }
}
