// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Arm;

public class ArmAnalogUp  extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  private Arm arm = Arm.getInstance();

  public ArmAnalogUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      arm.armAnalogUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
