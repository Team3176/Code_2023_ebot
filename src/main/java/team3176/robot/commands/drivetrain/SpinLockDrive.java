package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class SpinLockDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private PIDController wController = new PIDController(1.0, 0.0, 0.0);


  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private double spinLockAngle;

  public SpinLockDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(true);
    this.spinLockAngle = drivetrain.getPoseYawWrapped().getDegrees();

    //drivetrain.setCoastMode();
  }

  @Override
  public void execute() {

    drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND *0.7, 
                      strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND *0.7, 
                      MathUtil.clamp(wController.calculate(drivetrain.getPoseYawWrapped().getDegrees(), this.spinLockAngle), -2, 2));
    //spinCommand.getAsDouble()*100);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) { 
    drivetrain.setSpinLock(false);
   }
}