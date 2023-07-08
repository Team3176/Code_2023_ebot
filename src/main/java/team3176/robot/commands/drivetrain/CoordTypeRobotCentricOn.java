package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;


public class CoordTypeRobotCentricOn extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public CoordTypeRobotCentricOn() { 
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    drivetrain.setSpinLock(false);
    //drivetrain.setCoastMode();
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() { return true; }
}