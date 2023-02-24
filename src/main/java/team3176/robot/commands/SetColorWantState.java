// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.RobotConstants.Status;
import team3176.robot.subsystems.RobotState;

public class SetColorWantState extends CommandBase {
  /** Creates a new SetColorWantState. */
  RobotState m_RobotState;
  int LEDState;
  public SetColorWantState(int LEDState) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RobotState = RobotState.getInstance();
    addRequirements(m_RobotState);
    this.LEDState = LEDState;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //System.out.println("SetColorWantStateCommand()");
    m_RobotState.setColorWantState(LEDState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
