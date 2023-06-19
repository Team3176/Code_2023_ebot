// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.intakecube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.IntakeCube;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm;

public class PoopCube extends CommandBase {
  /** Creates a new IntakeCubeSpit. */
  IntakeCube m_IntakeCube = IntakeCube.getInstance();
  Claw m_Claw = Claw.getInstance();
  Arm m_Arm = Arm.getInstance();
  public PoopCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeCube, m_Arm, m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_ZERO_POS);
    m_IntakeCube.spinConveyor(0.4);
    m_IntakeCube.spinIntake(1);
    m_Claw.setClawMotor(-0.6, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_IntakeCube.spinConveyor(0.6);
    m_IntakeCube.spinIntake(1);
    m_Claw.setClawMotor(-0.6, 5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_IntakeCube.spinConveyor(0);
    m_IntakeCube.spinIntake(0);
    m_Claw.setClawMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
