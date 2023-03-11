// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import team3176.robot.constants.Hardwaremap;
import edu.wpi.first.wpilibj.DigitalInput;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeIO{
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isLinebreak = true;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public boolean isExtended = false;

    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("isLinebreak", isLinebreak);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("isExtended", isExtended);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      isLinebreak = table.getBoolean("isLinebreak", isLinebreak);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
      isExtended = table.getBoolean("isExtended", isExtended);
    }
  }

  public static class IntakeHardware
  {
    private TalonFX rollermotor = new TalonFX(Hardwaremap.intake_CID);
    private DoubleSolenoid pistonOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 6);
    private DigitalInput linebreak = new DigitalInput(8);

    public void setVelocity(double velocity)
    {
      rollermotor.configPeakOutputReverse(-velocity);
      rollermotor.set(ControlMode.PercentOutput, velocity);
    }

    public void setNeutralMode(int mode)
    {
      if (mode == 1)
      {
        rollermotor.setNeutralMode(NeutralMode.Coast);
      }
      else if (mode == 2)
      {
        rollermotor.setNeutralMode(NeutralMode.Brake);
      }
    }

    public void PistonState(boolean isExtended)
    {
      if (isExtended)
      {
        pistonOne.set(Value.kForward);
      }
      else
      {
        pistonOne.set(Value.kReverse);
      }
    }
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void setIsExtended(boolean isExtended) {}
}
