// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Arm;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.Intake;
import team3176.robot.subsystems.superstructure.Superstructure;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class PoopCube extends CommandBase {
  /** Creates a new ClawInhale. */
  Claw m_Claw = Claw.getInstance();
  Arm m_Arm = Arm.getInstance();
  Intake m_Intake = Intake.getInstance();
  Superstructure m_Superstructure = Superstructure.getInstance();
  Double CarryDeadband = 5.0;
  Double currentArmPosition;
  Double kArmPoopUpperLimit, kArmPoopLowerLimit, kArmCarryUpperLimit, kArmCarryLowerLimit;

  public PoopCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);
    addRequirements(m_Intake);
    addRequirements(m_Arm);
    addRequirements(m_Superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("PoopCube Init");
    //m_Intake.extendAndFreeSpin();
    currentArmPosition = m_Arm.hardware.getAbsolutePosition();
    kArmPoopLowerLimit = SuperStructureConstants.ARM_ZERO_POS - this.CarryDeadband;
    kArmPoopUpperLimit = SuperStructureConstants.ARM_ZERO_POS + this.CarryDeadband;
    kArmCarryLowerLimit = SuperStructureConstants.ARM_CARRY_POS - this.CarryDeadband;
    kArmCarryUpperLimit = SuperStructureConstants.ARM_CARRY_POS + this.CarryDeadband;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //System.out.println("PoopCube Exec" + kArmPoopLowerLimit + ", " + kArmPoopUpperLimit + ", " + kArmCarryLowerLimit + ", " + kArmCarryUpperLimit);
    //m_Intake.extendAndFreeSpin();
    m_Superstructure.preparePoop();
    new WaitCommand(2);
    //if (m_Arm.getArmPosition() >= kArmPoopLowerLimit && m_Arm.getArmPosition() <= kArmPoopUpperLimit) {
      m_Claw.scoreGamePiece();
    //}
    if (m_Claw.hardware.getLinebreakOne() == false || m_Claw.hardware.getLinebreakTwo() == false) {
      m_Claw.idle();
      new WaitCommand(2);
      m_Superstructure.prepareCarry();
    }
    this.currentArmPosition = m_Arm.hardware.getAbsolutePosition();
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("PoopCube End");
    //m_Intake.Retract();
    //m_Intake.spinVelocityPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("PoopCube IsFinished");
    //if (currentArmPosition >= kArmCarryLowerLimit && currentArmPosition <= kArmCarryUpperLimit) {
    //  return true;
    //} else{
      return false;
    //}
  }
}
