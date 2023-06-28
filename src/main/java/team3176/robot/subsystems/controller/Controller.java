// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.controller;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team3176.robot.constants.ControllerConstants;
// import team3176.robot.util.XboxController.XboxAxisAsButton;
// import team3176.robot.util.XboxController.*;

public class Controller {
  private static Controller instance = new Controller();

  public static Controller getInstance() {
    if (instance == null ) {
      instance = new Controller();
    }
      return instance;}

  /* The Three Physical Controllers that we have */
  
  public final CommandJoystick transStick;
  public final CommandJoystick rotStick;
  public final CommandXboxController operator;

  /* First Part of Creating the Buttons on the Joysticks */

  public Controller() {
    /* Finish Creating the Objects */

    transStick = new CommandJoystick(ControllerConstants.TRANS_ID);
    rotStick = new CommandJoystick(ControllerConstants.ROT_ID);
    operator = new CommandXboxController(ControllerConstants.OP_ID);

   
    
    
    /**
     * The DPAD of the Xbox Controller
     * The values are 0 as UP going in a 360 circle CCW
     */

    // op_DPAD_Up = new POVButton(operator, 0);
    // op_DPAD_Right = new POVButton(operator, 90);
    // op_DPAD_Down = new POVButton(operator, 180);
    // op_DPAD_Left = new POVButton(operator, 270);
  }

  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the Y axis of TransStick if it breaks deadband
   */

  public double getForward() {
    if(Math.abs(transStick.getY()) < 0.06) return 0.0;
    return ControllerConstants.FORWARD_AXIS_INVERSION * Math.pow(transStick.getY(), 1);
  }

  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the X axis of TransStick if it breaks deadband
   */

  public double getStrafe() {
    if(Math.abs(transStick.getX()) < 0.06) return 0.0;
    return ControllerConstants.STRAFE_AXIS_INVERSION * Math.pow(transStick.getX(), 1);
  }

  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the X axis of RotStick if it breaks deadband
   */

  public double getSpin() {
    if(Math.abs(rotStick.getX()) < 0.06) return 0.0;
    return ControllerConstants.SPIN_AXIS_INVERSION * rotStick.getX();
  }

  /**
   * Joystick response curve: Linear 
   * @return (f(x) = x) 
   */
  public static double joyResponseLinear(double joyInput) {
    return Math.pow(joyInput, 1);
  }

  /**
   * Joystick response curve: 8-2 Cubic 
   * @return (f(x) = 0.8x+0.2x^3)
   */
  public static double joyResponse82Cubic(double joyInput) {
    double a = 0.8;
    double b = 0.2;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
  
  /**
   * Joystick response curve: 6-4 Cubic 
   * @return (f(x) = 0.6x+0.4x^3)
   */
  public static double joyResponse64Cubic(double joyInput) {
    double a = 0.6;
    double b = 0.4;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
 
  /**
   * Joystick response curve: 4-6 Cubic 
   * @return (f(x) = 0.4x+0.6x^3)
   */
  public static double joyResponse46Cubic(double joyInput) {
    double a = 0.4;
    double b = 0.6;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
 
  /**
   * Joystick response curve: 2-8 Cubic 
   * @return (f(x) = 0.4x+0.6x^3)
   */
  public static double joyResponse28Cubic(double joyInput) {
    double a = 0.2;
    double b = 0.8;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
 
  /**
   * Joystick response curve: Full Cubic 
   * @return (f(x) = x^3)
   */
  public static double joyResponseFullCubic(double joyInput) {
    return (Math.pow(joyInput,3));
  }
  
  /**
   * Joystick response curve: 2parm25
   * @return (f(x) = a + (1 - a) * (b*x^3+b*x))
   * a = 0.2  
   * b = 0.5  
   */
  public static double joyResponse2parm25(double joyInput) {
    double a = 0.2;
    double b = 0.5;
    if (joyInput < 0) { 
      return ( -a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    } else {
      return ( a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    }
  }
  
  /**
   * Joystick response curve: 2parm28
   * @return (f(x) = a + (1 - a) * (b*x^3+b*x))
   * a = 0.2  
   * b = 0.8  
   */
  public static double joyResponse2parm28(double joyInput) {
    double a = 0.2;
    double b = 0.8;
    if (joyInput < 0) { 
      return ( -a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    } else {
      return ( a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    }
  }
  
  

  





  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the Y axis of RotStick if it breaks deadband
   */

  public double getOrbitSpeed() { //TODO: FIND IF WE NEED
    if(Math.abs(rotStick.getY()) < 0.06) return 0.0;
    return Math.pow(rotStick.getY(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The value of the y axis of the left joystick of the Xbox Controller
   */

  public double getOp_LeftY() {
    if(Math.abs(operator.getLeftY()) < 0.06) return 0.0;
    return Math.pow(operator.getLeftY(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The value of the x axis of the left joystick of the Xbox Controller
   */

  public double getOp_LeftX() {
    if(Math.abs(operator.getLeftX()) < 0.06) return 0.0;
    return Math.pow(operator.getLeftX(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The value of the y axis of the right joystick of the Xbox Controller
   */

  public double getOp_RightY() {
    if(Math.abs(operator.getRightY()) < 0.06) return 0.0;
    return Math.pow(operator.getLeftY(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The scales value of the x axis of the right joystick of the Xbox Controller
   */

  public double getOp_RightX() {
    if(Math.abs(operator.getRightX()) < 0.06) return 0.0;
    return Math.pow(operator.getRightX(), 1);
  }



  // public Trigger getOp_A() {return op_A;}
  // public Trigger getOp_A_FS() {return op_A_Shift;}
  // public Trigger getOp_A_DS() {return op_A_Double_Shift;}
  // public Trigger getOp_B() {return op_B;}
  // public Trigger getOp_B_FS() {return op_B_Shift;}
  // public Trigger getOp_B_DS() {return op_B_Double_Shift;}
  // public Trigger getOp_X() {return op_X;}
  // public Trigger getOp_X_FS() {return op_X_Shift;}
  // public Trigger getOp_X_DS() {return op_X_Double_Shift;}
  // public Trigger getOp_Y() {return op_Y;}
  // public Trigger getOp_Y_FS() {return op_Y_Shift;}
  // public Trigger getOp_Y_DS() {return op_Y_Double_Shift;}
  // public Trigger getOp_Start() {return op_Start;}
  // public Trigger getOp_Start_FS() {return op_Start_Shift;}
  // public Trigger getOp_Start_DS() {return op_Start_Double_Shift;}
  // public Trigger getOp_Back() {return op_Back;}
  // public Trigger getOp_Back_FS() {return op_Back_Shift;}
  // public Trigger getOp_Back_DS() {return op_Back_Double_Shift;}
  // public Trigger getOp_LeftTrigger() {return op_LTrigger;}
  // public Trigger getOp_RightTrigger() {return op_RTrigger;}
  
  // public POVButton getOp_DPAD_UP() {return op_DPAD_Up;}
  // public POVButton getOp_DPAD_RIGHT() {return op_DPAD_Right;}
  // public POVButton getOp_DPAD_DOWN() {return op_DPAD_Down;}
  // public POVButton getOp_DPAD_LEFT() {return op_DPAD_Left;}
}
