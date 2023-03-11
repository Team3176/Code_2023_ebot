// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;

/** Template hardware interface for a closed loop subsystem. */
public interface ClawIO{
  /** Contains all of the input data received from hardware. */
  public static class ClawIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isLinebreakOne = true;
    public boolean isLinebreakTwo = true;
    public double currentAmps = 0.0;
    public double tempCelcius = 0.0;



    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("isLinebreakOne", isLinebreakOne);
      table.put("isLinebreakTwo", isLinebreakTwo);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      isLinebreakOne = table.getBoolean("isLinebreakOne", isLinebreakOne);
      isLinebreakTwo = table.getBoolean("isLinebreakTwo", isLinebreakTwo);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
    }
  }

  public static class ClawHardware
  {
    private CANSparkMax claw = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
    private DigitalInput linebreakOne = new DigitalInput(9);
    private DigitalInput linebreakTwo = new DigitalInput(7);

    public void setVelocity(double velocity)
    {
      claw.set(velocity);
    }

    public void setAmps(int amps)
    {
      claw.setSmartCurrentLimit(amps);
    }

    public double getCurrent()
    {
      return claw.getOutputCurrent();
    }

    public double getTemperature()
    {
      return claw.getMotorTemperature();
    }

    public boolean getLinebreakOne()
    {
        return linebreakOne.get();
    }
    
    public boolean getLinebreakTwo()
    {
        return linebreakTwo.get();
    }
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setAmps(double amps) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}
}
