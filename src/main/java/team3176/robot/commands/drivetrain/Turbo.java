package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;


public class Turbo extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();


  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  public Turbo( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(false);
    //drivetrain.setCoastMode();
  }

  @Override
  public void execute() {
    drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND * 1.0, 
    strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND * 1.0, 
    spinCommand.getAsDouble()*7);
  }

  @Override
  public boolean isFinished() { return false; }
}