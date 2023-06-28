// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.arm;


import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Arm;

public class ArmAnalogIdle  extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  private Arm arm = Arm.getInstance();

  public ArmAnalogIdle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      arm.idle();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return true;
  }
}
