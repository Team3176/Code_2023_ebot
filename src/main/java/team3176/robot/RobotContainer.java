// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.arm.*;
import team3176.robot.commands.arm.manuallyPositionArm;
import team3176.robot.commands.autons.*;
import team3176.robot.commands.claw.*;
import team3176.robot.commands.claw.ClawInhaleCube;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.commands.drivetrain.SwerveDrive;
import team3176.robot.commands.drivetrain.SwerveDefense;
import team3176.robot.commands.intake.*;
import team3176.robot.commands.vision.*;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.RobotState;
import team3176.robot.subsystems.superstructure.Arm;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.Intake;
import team3176.robot.subsystems.superstructure.Superstructure;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Arm m_Arm;
  private final Controller m_Controller;
  private final Claw m_Claw;
  private final Intake m_Intake;
  //private final Compressor m_Compressor;
  private final Drivetrain m_Drivetrain;
  private final RobotState m_RobotState;
  private SendableChooser<String> m_autonChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Arm = Arm.getInstance();
    m_Controller = Controller.getInstance();
    m_Claw = Claw.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    m_Intake = Intake.getInstance();
    m_RobotState = RobotState.getInstance();
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
          () -> m_Controller.getForward(),
          () -> m_Controller.getStrafe(),
          () -> m_Controller.getSpin()));

    File paths = new File(Filesystem.getDeployDirectory(), "pathplanner");
    for(File f:paths.listFiles()){
      if(!f.isDirectory()) {
        String s= f.getName().split(".",1)[0];
        m_autonChooser.addOption(s, s);
      }
    }
    
    //m_autonChooser.setDefaultOption("cube_balance", "cube_balance");
    SmartDashboard.putData("Auton Choice", m_autonChooser);
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_Controller.getTransStick_Button1().whileTrue(new InstantCommand( () -> m_Drivetrain.setTurbo(true), m_Drivetrain));
    m_Controller.getTransStick_Button1().onFalse(new InstantCommand( () -> m_Drivetrain.setTurbo(false), m_Drivetrain));
    m_Controller.getTransStick_Button2().whileTrue(new InstantCommand( () -> m_Drivetrain.resetFieldOrientation(), m_Drivetrain));
    m_Controller.getTransStick_Button3().whileTrue(m_Claw.scoreGamePiece());
    //m_Controller.getTransStick_Button4().whileTrue(new PickupProtocol());
    m_Controller.getTransStick_Button3().whileTrue(new InstantCommand( () -> m_Drivetrain.setDriveMode(driveMode.DEFENSE), m_Drivetrain));
    m_Controller.getTransStick_Button3().onFalse(new InstantCommand( () -> m_Drivetrain.setDriveMode(driveMode.DRIVE), m_Drivetrain));
   
    //m_Controller.getRotStick_Button1().whileTrue(new TurtleSpeed());
    //m_Controller.getRotStick_Button2().whileTrue(new FlipField);
    m_Controller.getRotStick_Button4().whileTrue(new InstantCommand( () -> m_Drivetrain.setCoordType(coordType.ROBOT_CENTRIC), m_Drivetrain));
    m_Controller.getRotStick_Button4().onFalse(new InstantCommand( () -> m_Drivetrain.setCoordType(coordType.FIELD_CENTRIC), m_Drivetrain));
    //m_Controller.getRotStick_Button4().whileTrue(new SpinLock());

    m_Controller.operator.a().onTrue(m_RobotState.setColorWantStateCommand());

    //m_Controller.operator.povUp().onTrue(new ArmToHighPosition());
    //m_Controller.operator.povRight().onTrue(new ArmToCarryPosition());
    //m_Controller.operator.povDown().onTrue(new ArmToLowPosition());
    //m_Controller.operator.povLeft().onTrue(new ArmToMidPosition());

    //m_Controller.operator.start().onTrue(new ToggleVisionLEDs());
    //m_Controller.operator.back().onTrue(new SwitchToNextVisionPipeline());

    //m_Controller.operator.b().onTrue(new ClawInhaleCone());
    m_Controller.operator.b().whileTrue(new ClawInhaleCone());
    //m_Controller.operator.x().onTrue(new ClawInhaleCube());
    m_Controller.operator.x().whileTrue(new ClawInhaleCube());
    m_Controller.operator.a().whileTrue(new IntakeExtendSpin());
    m_Controller.operator.a().onFalse(new IntakeRetractSpinot ());
    m_Controller.operator.b().onTrue(m_RobotState.setColorWantStateCommand());

    //m_Controller.operator.a().whileTrue(new PickupProtocol());
    //m_Controller.operator.b().onTrue(AskForCone());
    //m_Controller.operator.x().onTrue(AskForCube());

    //m_Controller.operator.leftBumper().whileTrue(new manuallyPositionArm( () -> m_Controller.operator.getLeftY()));
    m_Controller.operator.leftBumper().whileTrue(new armAnalogDown());
    m_Controller.operator.leftBumper().onFalse(new armAnalogIdle());

    m_Controller.operator.rightBumper().whileTrue(new armAnalogUp());
    m_Controller.operator.rightBumper().onFalse(new armAnalogIdle());

    m_Controller.operator.y().whileTrue(m_Claw.scoreGamePiece());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String chosen = m_autonChooser.getSelected();

    
    PathPlannerAuto PPSwerveauto = new PathPlannerAuto(chosen);
    return PPSwerveauto.getauto();
  }
}
