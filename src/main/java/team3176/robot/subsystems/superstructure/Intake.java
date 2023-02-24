/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import team3176.robot.constants.Hardwaremap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX rollermotor = new TalonFX(Hardwaremap.intake_CID);
  private DoubleSolenoid pistonOne;
  private DoubleSolenoid pistonTwo;
  private DigitalInput linebreak;

  private boolean isExtended;
  private boolean isInIntake;
  private static Intake instance;
  public Intake() 
  {
    pistonOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 6);
    //pistonTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 2);
    linebreak = new DigitalInput(8);

  }

  public void spinVelocityPercent(double pct) {
    rollermotor.configPeakOutputReverse(-pct);
    rollermotor.set(ControlMode.PercentOutput, -pct);
    
  }

  public void setCoastMode() {
    rollermotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    rollermotor.setNeutralMode(NeutralMode.Brake);
  } 

  public void Extend() {
    pistonOne.set(Value.kForward);
    //pistonTwo.set(Value.kForward);
    this.isExtended = true;
  }

  public void Retract() {
    pistonOne.set(Value.kReverse);
    //pistonTwo.set(Value.kReverse);
    this.isExtended = false;
  }

  public boolean getLinebreak()
  {
    return linebreak.get();
  }

  public static Intake getInstance(){
    if ( instance == null ) {
      instance = new Intake();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Code stating if something is in the Intake
    // SmartDashboard.putBoolean("isInIntake", isInIntake);
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

  public Command extendAndSpin() {
    return this.startEnd(() ->{
      this.Extend();
      this.spinVelocityPercent(.1);
    }, () -> {
      this.Retract();
      this.spinVelocityPercent(0.0);
    });
  }


  public Command extendAndFreeSpin() {
    return this.startEnd(() ->{
      this.Extend();
      this.setCoastMode();
    }, () -> {
      this.Retract();
      this.setBrakeMode();
    });
  }
}
