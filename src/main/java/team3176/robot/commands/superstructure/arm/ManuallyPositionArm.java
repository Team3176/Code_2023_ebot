// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.superstructure.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.superstructure.Arm;

public class ManuallyPositionArm extends CommandBase {
  /** Creates a new IntakeExtendSpin. */
  private Arm arm = Arm.getInstance();
  private DoubleSupplier analogInput;
  private Double analogInputDeadband;

  public ManuallyPositionArm(DoubleSupplier analogInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.analogInput = analogInput;
    this.analogInputDeadband = 0.01;

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double updatedAnalogInput = this.analogInput.getAsDouble(); 
    if (updatedAnalogInput > (0 + analogInputDeadband)) {
      arm.armAnalogUpCommand();
    } 

    if (updatedAnalogInput < (0 - analogInputDeadband)) {
      arm.armAnalogDownCommand();
    }
   
    //if ((updatedAnalogInput < (0 + analogInputDeadband)) && (updatedAnalogInput > (0 + analogInputDeadband))) {
    //  m_Arm.
    //}
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Double updatedAnalogInput = this.analogInput.getAsDouble(); 
    return (updatedAnalogInput < (0 + analogInputDeadband)) && (updatedAnalogInput > (0 - analogInputDeadband));
  }  
  
}
