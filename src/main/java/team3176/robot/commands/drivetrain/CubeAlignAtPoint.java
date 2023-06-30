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




public class CubeAlignAtPoint extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private VisionCubeChase visionCubeChase = VisionCubeChase.getInstance();
  private Claw claw = Claw.getInstance();


  //private DoubleSupplier forwardCommand;
  //private DoubleSupplier strafeCommand;
  //private DoubleSupplier spinCommand;
  private double forwardCommand;
  private double strafeCommand;
  private double splicingSpinCommand;
  private double smallNum;

  //PIDController txController = new PIDController(.01, 0, 0);
  //double tx;
  boolean tv; 
  boolean[] tvArray = {false, false, false, false, false, false, false, false, false, false};
  int arrayIdx;

  public CubeAlignAtPoint() {
    addRequirements(drivetrain);
    addRequirements(visionCubeChase);
    arrayIdx = 0;
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.CUBECHASE);
    drivetrain.setSpinLock(false);
    //drivetrain.setCoastMode();
    //this.tx = visionCubeChase.getTx();
    double smallNum = Math.pow(10, -5);
  }

  @Override
  public void execute() {
    drivetrain.drive(smallNum, smallNum, 1.0);
    tv = LimelightHelpers.getTV("limelight-three");
    tvArray[arrayIdx] = tv;
    if (arrayIdx == (tvArray.length + 1)) {arrayIdx = 0;} else {arrayIdx++;};
  }

  @Override
  public boolean isFinished() {
    boolean tvArrayTripwire = true;
    for (int idx = 0; idx < tvArray.length; idx++) {
      tv = LimelightHelpers.getTV("limelight-three");
      tvArray[idx] = tv;
    } 
    for (int idx = 0; idx < tvArray.length; idx++) {
      tv = LimelightHelpers.getTV("limelight-three");
      if (!tvArray[idx]) { 
        tvArrayTripwire = false; 
      };
    }
    if (tvArrayTripwire) { return true;} else {return false;}

  }

  @Override
  public void end(boolean interrupted) { 
    drivetrain.setDriveMode(driveMode.DRIVE);
   }
}