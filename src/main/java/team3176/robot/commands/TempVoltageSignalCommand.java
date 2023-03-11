// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.RobotState;

public class TempVoltageSignalCommand extends CommandBase {
  /** Creates a new TempVoltageSignalCommand. */
  RobotState m_RobotState;
  public TempVoltageSignalCommand() {
    m_RobotState = RobotState.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_RobotState);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_RobotState.BatteryGuage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
