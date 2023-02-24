// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import team3176.robot.subsystems.vision.Vision.LEDState;



public class RobotState extends SubsystemBase {

  private final RobotStateIO io;
  private final RobotStateIOInputs inputs = new RobotStateIOInputs();
  private static RobotState instance;
  private int wantedLEDState;
  private Claw m_Claw;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int i = 0;

  private int m_flashcounter = 0;
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
    wantedLEDState = 0;

    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(SignalingConstants.LEDLENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setSegment(int start, int end, int red, int green, int blue)
    {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setRGB(i, red , green, blue);
      }
    }

    public void setSegment(int start, int end, Color color)
    {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setLED(i, color);
      }
      // m_led.setData(m_ledBuffer);
    }

    public void setleftfrontlow(Status s)
    {
      leftfrontlowcolor = LookUpColor(s);
      leftfrontlowflash = LookUpFlash(s);
      setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP, leftfrontlowcolor);
      m_led.setData(m_ledBuffer);
    }
    public void setleftfronthigh(Status s)
    {
      leftfronthighcolor = LookUpColor(s);
      leftfronthighflash = LookUpFlash(s);
      setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP, leftfronthighcolor);
      m_led.setData(m_ledBuffer);
    }
    public void setcrossbarhigh(Status s)
    {
      crosshighcolor = LookUpColor(s);
      crosshighflash = LookUpFlash(s);
      setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTOP, crosshighcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setrightfronthigh(Status s)
    {
      rightfronthighcolor = LookUpColor(s);
      rightfronthighflash = LookUpFlash(s);
      setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, rightfronthighcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setrightfrontlow(Status s)
    {
      rightfrontlowcolor = LookUpColor(s);
      rightfrontlowflash = LookUpFlash(s);
      setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP, rightfrontlowcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setleftback(Status s)
    {
      leftbackcolor = LookUpColor(s);
      leftbackflash = LookUpFlash(s);
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP, leftbackcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setcrossbarlow(Status s)
    {
      crosslowcolor = LookUpColor(s);
      crosslowflash = LookUpFlash(s);
      setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, crosslowcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setrightback(Status s)
    {
      rightbackcolor = LookUpColor(s);
      rightbackflash = LookUpFlash(s);
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP, rightbackcolor);
      m_led.setData(m_ledBuffer);
    }

  private Color LookUpColor(Status s){
    Color c = Color.kBlack;
    switch(s){
      case STABLE: c = Color.kGreen;
      break;
      case OK: c = Color.kDarkRed;
      break;
      case OPTIONALCHECK: c = Color.kLightYellow;
      break;
      case WARNING: c = Color.kFuchsia;
      break;
      case GOOD: c = Color.kLightCoral;
      break;
      case ERROR: c = Color.kLime;
      break;
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

  public void FlashColor()
  {
    m_flashcounter++;
    if (m_flashcounter == 25){
     if (leftfrontlowflash){
      setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP,leftfrontlowcolor);
     }
     if (leftfronthighflash){
      setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP,leftfrontlowcolor);
     }
     if (crosshighflash){
      setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTART,crosshighcolor);
     }
     if (rightfronthighflash){
      setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, rightfronthighcolor);
     }
     if (rightfrontlowflash){
      setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP,rightfrontlowcolor);
     }
     if (leftbackflash){
      setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP,leftbackcolor);
     }
     if (crosslowflash){
      setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, crosslowcolor);
     }
     if (rightbackflash){
      setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP,rightbackcolor);
     }
     m_led.setData(m_ledBuffer);
     }
    if (m_flashcounter == 50){
      if (leftfrontlowflash){
        setSegment(SignalingConstants.LEFTFRONTLOWSTART, SignalingConstants.LEFTFRONTLOWSTOP,Color.kBlack);
       }
       if (leftfronthighflash){
        setSegment(SignalingConstants.LEFTFRONTHIGHSTART, SignalingConstants.LEFTFRONTHIGHSTOP,Color.kBlack);
       }
       if (crosshighflash){
        setSegment(SignalingConstants.CROSSHIGHSTART, SignalingConstants.CROSSHIGHSTART,Color.kBlack);
       }
       if (rightfronthighflash){
        setSegment(SignalingConstants.RIGHTFRONTHIGHSTART, SignalingConstants.RIGHTFRONTHIGHSTOP, Color.kBlack);
       }
       if (rightfrontlowflash){
        setSegment(SignalingConstants.RIGHTFRONTLOWSTART, SignalingConstants.RIGHTFRONTLOWSTOP,Color.kBlack);
       }
       if (leftbackflash){
        setSegment(SignalingConstants.LEFTBACKSTART, SignalingConstants.LEFTBACKSTOP,Color.kBlack);
       }
       if (crosslowflash){
        setSegment(SignalingConstants.CROSSLOWSTART, SignalingConstants.CROSSLOWSTOP, Color.kBlack);
       }
       if (rightbackflash){
        setSegment(SignalingConstants.RIGHTBACKSTART, SignalingConstants.RIGHTBACKSTOP,Color.kBlack);
       }
       m_led.setData(m_ledBuffer);
      m_flashcounter = 0;
     }
  }


  public void update() {
    if (DriverStation.isFMSAttached() && (alliance == null)) {
      alliance = DriverStation.getAlliance();
    }
  }

  public void setColorWantState(int LEDState)
  {
    System.out.println("WAS CALLED");
    wantedLEDState = LEDState;
    if (wantedLEDState == 0)
    {
      setleftfrontlow(Status.NONE);
      setleftfronthigh(Status.NONE);
      setcrossbarhigh(Status.NONE);
      setrightfronthigh(Status.NONE);
      setrightfrontlow(Status.NONE);
      setleftback(Status.NONE);
      setcrossbarlow(Status.NONE);
      setrightback(Status.NONE);
    }
    else if (wantedLEDState == 1)
    {
      setleftfrontlow(Status.CONEFLASH);
      setleftfronthigh(Status.CONEFLASH);
      setcrossbarhigh(Status.CONEFLASH);
      setrightfronthigh(Status.CONEFLASH);
      setrightfrontlow(Status.CONEFLASH);
      setleftback(Status.CONEFLASH);
      setcrossbarlow(Status.CONEFLASH);
      setrightback(Status.CONEFLASH);
    }
    else if (wantedLEDState == 2)
    {
      setleftfrontlow(Status.CUBEFLASH);
      setleftfronthigh(Status.CUBEFLASH);
      setcrossbarhigh(Status.CUBEFLASH);
      setrightfronthigh(Status.CUBEFLASH);
      setrightfrontlow(Status.CUBEFLASH);
      setleftback(Status.CUBEFLASH);
      setcrossbarlow(Status.CUBEFLASH);
      setrightback(Status.CUBEFLASH);
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

    
    if ((m_Claw.getLinebreakOne() == false || m_Claw.getLinebreakTwo() == false) && i == 25)
    {
      System.out.println("AHHHHH");
      if (wantedLEDState == 1)
      {
        setleftfrontlow(Status.CONE);
        setleftfronthigh(Status.CONE);
        setrightfronthigh(Status.CONE);
        setrightfrontlow(Status.CONE);
        setleftback(Status.CONE);
        setcrossbarlow(Status.CONE);
        setrightback(Status.CONE);
      }
      else if (wantedLEDState == 2)
      {
        setleftfrontlow(Status.CUBE);
        setleftfronthigh(Status.CUBE);
        setrightfronthigh(Status.CUBE);
        setrightfrontlow(Status.CUBE);
        setleftback(Status.CUBE);
        setcrossbarlow(Status.CUBE);
        setrightback(Status.CUBE);
      }
      if (m_Claw.getLinebreakOne() == true && m_Claw.getLinebreakTwo() == true)
      {
        setleftfrontlow(Status.NONE);
        setleftfronthigh(Status.NONE);
        setrightfronthigh(Status.NONE);
        setrightfrontlow(Status.NONE);
        setleftback(Status.NONE);
        setcrossbarlow(Status.NONE);
        setrightback(Status.NONE);
        wantedLEDState = 0;
      }
      i=0;
    } else {
      i++;
    }
    
      FlashColor();
  }

  @Override
  public void simulationPeriodic() {}
}