package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;

// A totally bullshit way to delay execution of non-parallel commands in Auto

public class BlockingDelay extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public BlockingDelay() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) { 
   }
}