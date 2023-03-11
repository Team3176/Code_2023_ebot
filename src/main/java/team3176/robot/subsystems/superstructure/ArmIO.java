// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

/** Template hardware interface for a closed loop subsystem. */
public interface ArmIO{
  /** Contains all of the input data received from hardware. */
  public static class ArmIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};



    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("Position", position);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      position = table.getDouble("Position", position);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  public static class ArmHardware
  {
    private CANSparkMax armController = new CANSparkMax(Hardwaremap.arm_CID, MotorType.kBrushless);
    private CANCoder armEncoder = new CANCoder(Hardwaremap.armEncoder_CID);

    public void setVelocity(double percent)
    {
        armController.set(percent);
    }

    public void setVelocityPID(double turnOutput, double feedForward)
    {
        armController.set(turnOutput + feedForward);
    }

    public void setNeutralMode(int mode)
    {
        if (mode == 1)
        {
            armController.setIdleMode(IdleMode.kCoast);
        }
        else if (mode == 2)
        {
            armController.setIdleMode(IdleMode.kBrake);
        }
    }

    public void encoderConfig()
    {
        this.armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.armEncoder.configMagnetOffset(SuperStructureConstants.ARM_ENCODER_OFFSET);
        this.armEncoder.configSensorDirection(false,100);
    }

    public void setOpenLoopRampRate(double rate)
    {
        armController.setOpenLoopRampRate(rate);
    }

    public void flashMotor()
    {
        armController.burnFlash();
    }

    public double getAbsolutePosition()
    {
        return armEncoder.getAbsolutePosition();
    }
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void setPosition(double position) {}
}

