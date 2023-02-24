package team3176.robot.commands.arm;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Arm;

public class ArmFollowTrajectory  extends CommandBase {
    /** Creates a new IntakeExtendSpin. */
    private Arm m_Arm = Arm.getInstance();
    TrapezoidProfile traj;
    double goalAngle;
    Timer timeElapsed;

    public ArmFollowTrajectory(double goalAngle) {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_Arm);
      this.goalAngle = goalAngle;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        traj = new TrapezoidProfile(new Constraints(20, 20), 
                                    new State(goalAngle,0.0),
                                    new State(m_Arm.getArmPosition(),0.0));
        timeElapsed.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        State setpoint = traj.calculate(timeElapsed.get());
        m_Arm.setAngleSetpoint(setpoint.position);
    }
     
      
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return traj.isFinished(timeElapsed.get());
    }
  }