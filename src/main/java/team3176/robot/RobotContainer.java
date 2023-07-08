// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import java.io.File;


import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import team3176.robot.commands.*;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.commands.superstructure.claw.ClawIdle;
import team3176.robot.commands.superstructure.intakecube.*;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
import team3176.robot.subsystems.superstructure.Arm;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.IntakeCube;
import team3176.robot.subsystems.superstructure.IntakeCone;

import team3176.robot.subsystems.superstructure.Superstructure;
//import team3176.robot.subsystems.vision.VisionDual;
import team3176.robot.subsystems.vision.VisionCubeChase;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Arm arm;
  private final Controller controller;
  private final Claw claw;
  private final IntakeCube intakeCube;
  private final IntakeCone intakeCone;
  private PowerDistribution pdh; 

  
  // is this why we don't have a compressor? private final Compressor m_Compressor
  private final Drivetrain drivetrain;
  private final VisionCubeChase vision;
  private final Superstructure superstructure;
  private SendableChooser<String> autonChooser;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    arm = Arm.getInstance();
    controller = Controller.getInstance();
    claw = Claw.getInstance();
    drivetrain = Drivetrain.getInstance();
    intakeCube = IntakeCube.getInstance();
    intakeCone = IntakeCone.getInstance();
    pdh = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kRev);

    vision = VisionCubeChase.getInstance();
    superstructure = Superstructure.getInstance();
    drivetrain.setDefaultCommand(new SwerveDrive(
        controller::getForward,
        controller::getStrafe,
        controller::getSpin));
    arm.setDefaultCommand(arm.armFineTune( () -> controller.operator.getLeftY()));
    autonChooser = new SendableChooser<>();
    File paths = new File(Filesystem.getDeployDirectory(), "pathplanner");
    for (File f : paths.listFiles()) {
      if (!f.isDirectory()) {
        String s = f.getName().split("\\.", 0)[0];
        autonChooser.addOption(s, s);
      }
    }

  
    SmartDashboard.putData("Auton Choice", autonChooser);
    
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    controller.transStick.button(1).whileTrue(claw.scoreGamePiece());
    //m_Controller.getTransStick_Button1().onFalse(new InstantCommand(() -> m_Drivetrain.setTurbo(false), m_Drivetrain));
    controller.transStick.button(2).whileTrue(new IntakeGroundCubeGuided());
    controller.transStick.button(2).onFalse(new IntakeRetractSpinot().andThen(superstructure.prepareCarry()));
    controller.transStick.button(2).onFalse(superstructure.prepareCarry());
    controller.transStick.button(3).whileTrue(new SetColorWantState(3));
    controller.transStick.button(3).whileTrue(superstructure.groundCube());
    controller.transStick.button(3).onFalse(new IntakeRetractSpinot());
    controller.transStick.button(3).onFalse(superstructure.prepareCarry());

    controller.transStick.button(4).whileTrue(superstructure.prepareScoreHigh());
    controller.transStick.button(4).onFalse((superstructure.prepareCarry()));
//    controller.transStick.button(5).onTrue(new InstantCommand(drivetrain::resetPoseToVision,drivetrain));
    controller.transStick.button(10).whileTrue(new InstantCommand(drivetrain::setBrakeMode).andThen(new SwerveDefense()));
     //m_Controller.getTransStick_Button10()
     //    .onFalse(new InstantCommand(() -> m_Drivetrain.setDriveMode(driveMode.DRIVE), m_Drivetrain));

    //m_Controller.getRotStick_Button2().whileTrue(new FlipField);
    controller.transStick.button(14).and(controller.transStick.button(15).onTrue(new CoordTypeFieldCentricOn()));
    controller.transStick.button(14).and(controller.transStick.button(16).onTrue(new CoordTypeRobotCentricOn()));

    
    
    //controller.rotStick.button(1).whileTrue(new CubeChase(
    controller.rotStick.button(1).whileTrue(new Turbo(
      controller::getForward,
      controller::getStrafe,
      controller::getSpin
    ));
 
    controller.rotStick.button(2).whileTrue(new SpinLockDrive(
      controller::getForward,
      controller::getStrafe)
    ); 
    

    controller.rotStick.button(3).whileTrue(new InstantCommand(drivetrain::setBrakeMode).andThen(new SwerveDefense()));

    controller.rotStick.button(4).whileTrue(superstructure.intakeCubeHumanPlayer());
    controller.rotStick.button(4).onFalse(superstructure.prepareCarry());
    controller.rotStick.button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));

    double conveyorBumpTime = .1;  //In units of seconds
    controller.operator.povUp().whileTrue(superstructure.prepareScoreHigh());
    controller.operator.povUp().onTrue(intakeCube.bumpConveyor().withTimeout(conveyorBumpTime));
    controller.operator.povRight().whileTrue(superstructure.prepareCarry());
    controller.operator.povRight().onTrue(intakeCube.bumpConveyor().withTimeout(conveyorBumpTime));
    controller.operator.povDown().whileTrue(superstructure.prepareCatch());
    controller.operator.povDown().onTrue(intakeCube.bumpConveyor().withTimeout(conveyorBumpTime));
    controller.operator.povLeft().whileTrue(superstructure.prepareScoreMid());
    controller.operator.povLeft().onTrue(intakeCube.bumpConveyor().withTimeout(conveyorBumpTime));

    // m_Controller.operator.start().onTrue(new ToggleVisionLEDs());
    // m_Controller.operator.back().onTrue(new SwitchToNextVisionPipeline());

    controller.operator.b().onTrue(new SetColorWantState(1));
    controller.operator.b().whileTrue(superstructure.intakeConeHumanPlayer());
    controller.operator.b().onFalse(superstructure.prepareCarry());
    
    controller.operator.x().onTrue(new SetColorWantState(2));
    controller.operator.x().whileTrue(superstructure.intakeCubeHumanPlayer());
    controller.operator.x().onFalse(superstructure.prepareCarry());

    controller.operator.a().onTrue(new SetColorWantState(3));
    controller.operator.a().whileTrue(superstructure.groundCube());
    controller.operator.a().onFalse(new IntakeRetractSpinot());
    controller.operator.a().onFalse(superstructure.prepareCarry());

    controller.operator.y().onTrue(new SetColorWantState(0));
    controller.operator.y().whileTrue(claw.scoreGamePiece());
    controller.operator.y().onFalse(new ClawIdle());


    controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).onTrue(new SetColorWantState(3));
    controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).whileTrue(new IntakeGroundCube());
    controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).onFalse(new IntakeRetractSpinot());
    //m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).onFalse(m_Superstructure.prepareCarry());
    
    controller.operator.leftBumper().and(controller.operator.rightBumper()).whileTrue((new PoopCube()));
//    m_Controller.operator.leftBumper().and(m_Controller.operator.rightBumper()).onFalse(new InstantCommand( () -> m_IntakeCone.idle())); 
    

    
    controller.operator.leftTrigger().onTrue(arm.armSetPositionOnce(140).andThen(arm.armFineTune( () -> controller.operator.getLeftY())));
    //m_Controller.operator.leftBumper().onTrue(m_Arm.armSetPositionOnce(200).andThen(m_Arm.armFineTune( () -> m_Controller.operator.getLeftY())));
    //m_Controller.operator.leftBumper().onTrue(new ArmFollowTrajectory(SuperStructureConstants.ARM_MID_POS));
    //m_Controller.operator.start().whileTrue(new OldPoopCube());
    controller.operator.start().whileTrue(new InstantCommand(intakeCone::spit));
    controller.operator.start().onFalse(new InstantCommand(intakeCone::idle));
    //m_Controller.operator.start().onFalse(new IntakeRetractSpinot().andThen(m_Superstructure.prepareCarry()));
    controller.operator.back().whileTrue(superstructure.preparePoop());
    //m_Controller.operator.leftTrigger().whileTrue(new PoopCube()); 
    controller.operator.rightTrigger().whileTrue(superstructure.preparePoop());
  }

  public void setArmCoast() {
    arm.setCoastMode();
  }
  
  public void setArmBrake() {
    arm.setBrakeMode();
  }
  
  public void setThrustCoast() {
    drivetrain.setCoastMode();
  }

  public void setThrustBrake() {
    drivetrain.setBrakeMode();
  }

  public void setFieldCentric() {
    drivetrain.setCoordType(coordType.FIELD_CENTRIC);
  }
  public void clearCanFaults(){
    pdh.clearStickyFaults();
  }

  public void printCanFaults(){
    pdh.getStickyFaults();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String chosen = autonChooser.getSelected();
    //String chosen = "wall_cone_exit_balance";

    PathPlannerAuto ppSwerveAuto = new PathPlannerAuto(chosen);
    return ppSwerveAuto.getauto();
  }
}
