package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.vision.VisionCubeChase;
import edu.wpi.first.math.controller.PIDController;
import team3176.robot.subsystems.drivetrain.LimelightHelpers;
import team3176.robot.subsystems.superstructure.Claw;




public class CubeChaseAuto extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private VisionCubeChase visionCubeChase = VisionCubeChase.getInstance();
  private Claw claw = Claw.getInstance();


  //private DoubleSupplier forwardCommand;
  //private DoubleSupplier strafeCommand;
  //private DoubleSupplier spinCommand;
  private double forwardCommand;
  private double strafeCommand;
  private double splicingSpinCommand;

  //PIDController txController = new PIDController(.01, 0, 0);
  //double tx;
  //boolean tv; 

  public CubeChaseAuto() {
    addRequirements(drivetrain);
    addRequirements(visionCubeChase);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.CUBECHASE);
    drivetrain.setSpinLock(false);
    //drivetrain.setCoastMode();
    //this.tx = visionCubeChase.getTx();
  }

  @Override
  public void execute() {
    //this.tx = visionCubeChase.getTx();
    //System.out.println("tx: "+ this.tx + ", splicingSpinCommand: " + splicingSpinCommand);
  }

  @Override
  public boolean isFinished() {
    if (claw.getLinebreakOne()) {
      return true;
    } else return false;
  }

  @Override
  public void end(boolean interrupted) { 
    drivetrain.setDriveMode(driveMode.DRIVE);
   }
}