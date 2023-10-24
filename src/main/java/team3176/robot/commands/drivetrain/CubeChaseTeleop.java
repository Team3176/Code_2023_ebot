package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.vision.VisionCubeChase;
import edu.wpi.first.math.controller.PIDController;
import team3176.robot.subsystems.drivetrain.LimelightHelpers;




public class CubeChaseTeleop extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private VisionCubeChase visionCubeChase = VisionCubeChase.getInstance();


  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;
  private double splicingSpinCommand;

  PIDController txController = new PIDController(.01, 0, 0);
  double tx;

  public CubeChaseTeleop( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    addRequirements(drivetrain);
    addRequirements(visionCubeChase);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(false);
    //drivetrain.setCoastMode();
    //this.tx = visionCubeChase.getTx();
  }

  @Override
  public void execute() {
    //this.tx = visionCubeChase.getTx();
    this.tx = LimelightHelpers.getTX("limelight-three");
    splicingSpinCommand = 1 * txController.calculate(tx, 0.0);
    System.out.println("tx: "+ this.tx + ", splicingSpinCommand: " + splicingSpinCommand);
    //drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND * 1.0, 
    //#strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND * 1.0, 
    //spinCommand.getAsDouble()*7);
    drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND * 1.0, 
    strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND * 1.0, 
    splicingSpinCommand *7);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}