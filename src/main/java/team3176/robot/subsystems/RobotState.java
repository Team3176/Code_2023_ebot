// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3176.robot.subsystems.RobotStateIO; 
import team3176.robot.subsystems.RobotStateIO.RobotStateIOInputs; 

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import team3176.robot.constants.RobotConstants.Status;
import team3176.robot.constants.SignalingConstants;

import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.vision.Vision.LEDState;



public class RobotState extends SubsystemBase {

  private final RobotStateIO io;
  private final RobotStateIOInputs inputs = new RobotStateIOInputs();
  private static RobotState instance;
  private int wantedLEDState;
  private Claw m_Claw;
  private Drivetrain m_Drivetrain;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int i = 0;

  private double m_flashcounter = 0;
  private boolean leftfrontlowflash = false;
  private boolean leftfronthighflash = false;
  private boolean crosshighflash = false;
  private boolean rightfronthighflash = false;
  private boolean rightfrontlowflash = false;
  private boolean leftbackflash = false;
  private boolean crosslowflash = false;
  private boolean rightbackflash = false;
  private Color leftfrontlowcolor = Color.kBlack;
  private Color leftfronthighcolor = Color.kBlack;
  private Color crosshighcolor = Color.kBlack;
  private Color rightfronthighcolor = Color.kBlack;
  private Color rightfrontlowcolor = Color.kBlack;
  private Color leftbackcolor = Color.kBlack;
  private Color crosslowcolor = Color.kBlack;
  private Color rightbackcolor = Color.kBlack;
  private Color allcolor = Color.kBlack;
  private boolean allflash = false;
  private int voltage;
  private int tempVoltage = 14;

  private boolean isSolid;
  private boolean isFlashing;

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
    m_Claw = Claw.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    wantedLEDState = 0;
    isSolid = false;
    isFlashing = false;

    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(SignalingConstants.LEDLENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
    SmartDashboard.putNumber("BatteryVoltage", voltage);
  }

  // public void setSegment(int start, int end, int red, int green, int blue) {
  //     for (var i=start; i < end; i++)
  //     {
  //       m_ledBuffer.setRGB(i, red , green, blue);
  //     }
  // }

  public void setSegment(int start, int end, Color color) {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setLED(i, color);
      }
      // m_led.setData(m_ledBuffer);
  }

  // public void setleftfrontlow(Status s) {
  //     leftfrontlowcolor = LookUpColor(s);
  //     leftfrontlowflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP, leftfrontlowcolor);
  //     m_led.setData(m_ledBuffer);
  // }
  
  // public void setleftfronthigh(Status s) {
  //     leftfronthighcolor = LookUpColor(s);
  //     leftfronthighflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP, leftfronthighcolor);
  //     m_led.setData(m_ledBuffer);
  // }
  
  // public void setcrossbarhigh(Status s) {
  //     crosshighcolor = LookUpColor(s);
  //     crosshighflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP, crosshighcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setrightfronthigh(Status s) {
  //     rightfronthighcolor = LookUpColor(s);
  //     rightfronthighflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, rightfronthighcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setrightfrontlow(Status s) {
  //     rightfrontlowcolor = LookUpColor(s);
  //     rightfrontlowflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP, rightfrontlowcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setleftback(Status s) {
  //     leftbackcolor = LookUpColor(s);
  //     leftbackflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP, leftbackcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setcrossbarlow(Status s) {
  //     crosslowcolor = LookUpColor(s);
  //     crosslowflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, crosslowcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  // public void setrightback(Status s) {
  //     rightbackcolor = LookUpColor(s);
  //     rightbackflash = LookUpFlash(s);
  //     setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP, rightbackcolor);
  //     m_led.setData(m_ledBuffer);
  // }

  public void setallyellow()
  {
    if (isFlashing = true)
    {
      allflash = true;
    }
    setSegment(0, 73, Color.kOrange);
    m_led.setData(m_ledBuffer);
  }

  public void setallpurple()
  {
    if (isFlashing = true)
    {
      allflash = true;
    }
    setSegment(0, 73, Color.kPurple);
    m_led.setData(m_ledBuffer);
  }

  public void setallred()
  {
    if (isFlashing = true)
    {
      allflash = true;
    }
    setSegment(0, 73, Color.kRed);
    m_led.setData(m_ledBuffer);
  }

  public void setallblack()
  {
    setSegment(0, 73, Color.kBlack);
    m_led.setData(m_ledBuffer);
  }

  public void setall(Status s)
  {
    allcolor = LookUpColor(s);
    allflash = LookUpFlash(s);
    setSegment(0, 73, allcolor);
    m_led.setData(m_ledBuffer);
  }

  private Color LookUpColor(Status s){
    Color c = Color.kBlack;
    switch(s){
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case CONEFLASH: c = Color.kOrange;
      break;
      case CUBEFLASH: c = Color.kPurple;
      break;
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }
  
  private Boolean LookUpFlash(Status s){
      return ((s == Status.CUBEFLASH) || (s == Status.CONEFLASH));
  }

  // public void FlashColor() {
  //   m_flashcounter++;
  //   if (m_flashcounter == 25){
  //     if (leftfrontlowflash){
  //       setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP,leftfrontlowcolor);
  //     }
  //     if (leftfronthighflash){
  //       setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP,leftfrontlowcolor);
  //     }
  //     if (crosshighflash){
  //       setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP,crosshighcolor);
  //     }
  //     if (rightfronthighflash){
  //       setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, rightfronthighcolor);
  //     }
  //     if (rightfrontlowflash){
  //       setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP,rightfrontlowcolor);
  //     }
  //     if (leftbackflash){
  //       setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP,leftbackcolor);
  //     }
  //     if (crosslowflash){
  //       setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, crosslowcolor);
  //     }
  //     if (rightbackflash){
  //       setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP,rightbackcolor);
  //     }
  //    m_led.setData(m_ledBuffer);
  //    }
  //   if (m_flashcounter == 50){
  //     if (leftfrontlowflash){
  //       setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP,Color.kBlack);
  //     }
  //     if (leftfronthighflash){
  //       setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP,Color.kBlack);
  //     }
  //     if (crosshighflash){
  //       setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP,Color.kBlack);
  //     }
  //     if (rightfronthighflash){
  //       setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, Color.kBlack);
  //     }
  //     if (rightfrontlowflash){
  //       setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP,Color.kBlack);
  //     }
  //     if (leftbackflash){
  //       setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP,Color.kBlack);
  //     }
  //     if (crosslowflash){
  //       setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, Color.kBlack);
  //     }
  //     if (rightbackflash){
  //       setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP,Color.kBlack);
  //     }
  //     m_led.setData(m_ledBuffer);
  //     m_flashcounter = 0;
  //   }
  // }

  public void flashAll()
  {
    if (m_flashcounter == 10)
    {
      if (allflash = true)
      {
        if (wantedLEDState == 1)
        {
          setallyellow();
        }
        else if (wantedLEDState == 2)
        {
          setallpurple();
        }
        else if (wantedLEDState == 3)
        {
          setallred();
        }
      }
      m_led.setData(m_ledBuffer);
    }
    if (m_flashcounter == 20)
    {
      if (allflash = true)
      {
        setallblack();
      }
      m_led.setData(m_ledBuffer);
      m_flashcounter = 0;
    }
    m_flashcounter++;
  }


  public void update() {
    if (DriverStation.isFMSAttached() && (alliance == null)) {
      alliance = DriverStation.getAlliance();
    }
  }

  public void setColorWantState(int LEDState) {
    System.out.println("WAS CALLED");
    wantedLEDState = LEDState;
    if (wantedLEDState == 0) {
      isFlashing = false;
      setallblack();
    }
    else if (wantedLEDState == 1) {
      isFlashing = true;
      setallyellow();
    }
    else if (wantedLEDState == 2) {
      isFlashing = true;
      setallpurple();
    }
    else if (wantedLEDState == 3) {
      isFlashing = true;
      setallred();
    }
  }

  public void BatteryGuage()
  {
    // TEMP for loop for testing
    //double voltage = 13.1;
    // while (voltage >= -0.5)
    // {
      //voltage = voltage -0.1;
      //Timer.delay(0.1);
      setallblack();
      System.out.println("BatteryGuage()");
      if (voltage >= 13)
      {
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP, Color.kLimeGreen);
        //m_led.setData(m_ledBuffer);
        System.out.println("Voltage >= 13");
      }
      else if (voltage >= 12)
      {
        //setallblack();
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 1, Color.kLimeGreen);
        //m_led.setData(m_ledBuffer);
        System.out.println("Voltage >= 12");

      }
      else if (voltage >= 11)
      {
        //setallblack();
        System.out.println("Voltage >= 11");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 2, Color.kLimeGreen);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 10)
      {
        //setallblack();
        System.out.println("Voltage >= 10");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 3, Color.kLimeGreen);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 9)
      {
        //setallblack();
        System.out.println("Voltage >= 9");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 4, Color.kLimeGreen);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 8)
      {
        //setallblack();
        System.out.println("Voltage >= 8");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 5, Color.kDarkOrange);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 7)
      {
        //setallblack();
        System.out.println("Voltage >= 7");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 6, Color.kDarkOrange);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 6)
      {
        //setallblack();
        System.out.println("Voltage >= 6");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 7, Color.kDarkOrange);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 5)
      {
        //setallblack();
        System.out.println("Voltage >= 5");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 8, Color.kDarkOrange);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 4)
      {
        //setallblack();
        System.out.println("Voltage >= 4");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 9, Color.kDarkOrange);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 3)
      {
        //setallblack();
        System.out.println("Voltage >= 3");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 10, Color.kRed);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 2)
      {
        //setallblack();
        System.out.println("Voltage >= 2");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 11, Color.kRed);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage >= 1)
      {
        //setallblack();
        System.out.println("Voltage >= 1");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 12, Color.kRed);
        //m_led.setData(m_ledBuffer);
      }
      else if (voltage > 0)
      {
        //setallblack();
        System.out.println("Voltage > 0");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP - 13, Color.kRed);
        //m_led.setData(m_ledBuffer);
      }
      else
      {
        //setallblack();
        System.out.println("NO MORE VOLTAGE");
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP, Color.kRed);
        //m_led.setData(m_ledBuffer);
      }
      //m_led.setData(m_ledBuffer);
    // }
  }

  public void ChargeStationSignal()
  {
    double balance = m_Drivetrain.getChassisPitch();
    if (balance == 180)
    {
      setSegment(0, 73, Color.kBlack);
    }
    else if (balance >= 160)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 1, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 1, Color.kGreen);
    }
    else if (balance >= 144)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 2, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 2, Color.kGreen);
    }
    else if (balance >= 128)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 3, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 3, Color.kGreen);
    }
    else if (balance >= 112)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 4, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 4, Color.kGreen);
    }
    else if (balance >= 96)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 5, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 5, Color.kGreen);
    }
    else if (balance >= 80)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 6, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 6, Color.kGreen);
    }
    else if (balance >= 64)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 7, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 7, Color.kGreen);
    }
    else if (balance >= 48)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 8, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 8, Color.kGreen);
    }
    else if (balance >= 32)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 9, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 9, Color.kGreen);
    }
    else if (balance >= 16)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 10, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 10, Color.kGreen);
    }
    else if (balance == 0)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP, Color.kGreen);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP, Color.kGreen);
    }
    else if (balance <= -16)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 1, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 1, Color.kRed);
    }
    else if (balance <= -32)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 2, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 2, Color.kRed);
    }
    else if (balance <= -64)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 3, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 3, Color.kRed);
    }
    else if (balance <= -80)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 4, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 4, Color.kRed);
    }
    else if (balance <= -96)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 5, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 5, Color.kRed);
    }
    else if (balance <= -112)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 6, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 6, Color.kRed);
    }
    else if (balance <= -128)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 7, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 7, Color.kRed);
    }
    else if (balance <= -144)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 8, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 8, Color.kRed);
    }
    else if (balance <= -160)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 9, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 9, Color.kRed);
    }
    else if (balance <= -176)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTART + 10, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTART + 10, Color.kRed);
    }
    else if (balance == -180)
    {
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP, Color.kRed);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP, Color.kRed);
    }
  }

  // public Command setColorWantStateCommand(int LEDState)
  // {
  //   System.out.println("setColorWantStateCommand()");
  //   return this.runOnce(() -> setColorWantState(LEDState));
  // }

  public static RobotState getInstance() {
    if(instance == null) {instance = new RobotState(new RobotStateIO() {});}
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
    //voltage = (int) RobotController.getBatteryVoltage();
    voltage = (int) SmartDashboard.getNumber("BatteryVoltage", voltage);
    if (voltage < tempVoltage)
    {
      BatteryGuage();
      tempVoltage = voltage;
      System.out.println("TempVoltage: " + tempVoltage);
    }

    //(m_Claw.getLinebreakOne() == false || m_Claw.getLinebreakTwo() == false)
    if (m_Claw.getLinebreakOne() == false || m_Claw.getLinebreakTwo() == false) {
      isSolid = true;
      isFlashing = false;
      if (wantedLEDState == 1) {
        setallyellow();
      }
      else if (wantedLEDState == 2 || wantedLEDState == 3) {
        setallpurple();
      }
    }
    else if ((m_Claw.getLinebreakOne() == true && m_Claw.getLinebreakTwo() == true) && isSolid == true)
    {
      wantedLEDState = 0;
      isSolid = false;
    }
    else if (isSolid == false && isFlashing == true){
      if (wantedLEDState == 0)
      {
        allflash = false;
      }
      flashAll();
    }
  }

  @Override
  public void simulationPeriodic() {}
}