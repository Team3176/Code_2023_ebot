// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class ClawInhaleCube extends CommandBase {
  /** Creates a new ClawInhale. */
  Claw m_Claw = Claw.getInstance();
  Timer continueRunningTimer = new Timer();
  public ClawInhaleCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Claw.setCurrentGamePiece(GamePiece.CUBE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Claw.intake();
    if(m_Claw.getLinebreakOne()) {
      continueRunningTimer.restart();
    }
    System.out.println("ClawInhaleCube");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Claw.idle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return continueRunningTimer.get() > 0.5;
  }
}
