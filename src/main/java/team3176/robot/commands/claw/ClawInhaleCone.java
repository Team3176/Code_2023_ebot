// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.claw;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class ClawInhaleCone extends CommandBase {
  /** Creates a new ClawInhale. */
  Claw m_Claw = Claw.getInstance();
  Timer continueRunningTimer = new Timer();
  public ClawInhaleCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Claw.setCurrentGamePiece(GamePiece.CONE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Claw.intake();
    if(m_Claw.getLinebreakTwo()) {
      continueRunningTimer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Claw.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return continueRunningTimer.get() > 0.5;
  }
}
