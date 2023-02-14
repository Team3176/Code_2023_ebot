// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3176.robot.subsystems.RobotStateIO; 
import team3176.robot.subsystems.RobotStateIO.RobotStateIOInputs; 

import team3176.robot.subsystems.signalling.Signalling;
import team3176.robot.subsystems.superstructure.Intake;
import team3176.robot.constants.RobotConstants.Status;



public class RobotState extends SubsystemBase {

  private final RobotStateIO io;
  private final RobotStateIOInputs inputs = new RobotStateIOInputs();
  private static RobotState instance;
  private Signalling m_Signalling;
  private Intake m_Intake;

  private Alliance alliance; 

  enum e_ClawPositionState {
    OPEN,
    CLOSED,
    IDLE
  }

  enum e_ClawRollersState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Claw, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Claw, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_IntakePositionState {
    EXTENDED,
    RETRACTED
  }


  enum e_IntakeDetectsImHolding {
    CONE,
    CUBE,
    NOTHING,
    ERROR
  }

  enum e_IntakeMotorState {
    PosSPIN,  //Means front-most side of rollers spinning INTO the Intake, toward it's center 
    NegSPIN,  //Means front-most side of rollers spinning OUT-OF the Intake, toward it's exterior
    NoSPIN,  //Means front-most rollers are not spinning
  }

  enum e_ArmShoulderPositionState {
    IsUP,  //Means Elevator is in up postion
    IsDOWN  // Means Elevator is in down position
  }

  enum e_ArmElbowPositionState {
    IsPICKUP,   //Means Arms is in position to receive game element from Intake
    IsHIGHCONE,  // Means Arm is in High Position to deposit cone
    IsHIGHCUBE,  // Means Arm is in High Position to deposit cube
    IsMIDCONE,  // Means Arm is in Mid Position to deposit cone
    IsMIDCUBE,  // Means Arm is in Mid Position to deposit cube
    IsFLOORCONE,  // Means Arm is in Floor High Position to deposit cone 
    IsFLOORCUBE,  // Means Arm is in Floor Position to deposit cube
  }

  public enum e_CurrentGameElementImWanting{
    CONE,
    CUBE,
    NONE
  }

  public enum e_CurrentGameElementImHolding{
    CONE,
    CUBE,
    NONE
  }


  private RobotState(RobotStateIO io) {
    this.io = io;
    m_Signalling = Signalling.getInstance();
    m_Intake = Intake.getInstance();
  }

  public void update() {
    if (DriverStation.isFMSAttached() && (alliance == null)) {
      alliance = DriverStation.getAlliance();
    }
  }

  public static RobotState getInstance() {
    if(instance == null) {instance = new RobotState(new RobotStateIO() {});}
    return instance;
  }

  // public Command setColorStateCommand()
  // {
  //   return this.runOnce(if (m_Intake.isCone() == true));
  // }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
    if (m_Intake.isCone() == true)
    {
      m_Signalling.setleft(Status.CONE);
    }
    else if (m_Intake.isSquircle() == true)
    {
      m_Signalling.setleft(Status.CUBE);
    }
    else{m_Signalling.setleft(Status.NONE);}
  }

  @Override
  public void simulationPeriodic() {}
}