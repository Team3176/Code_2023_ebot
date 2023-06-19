// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import java.io.File;

import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.*;
import team3176.robot.commands.autons.*;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.commands.superstructure.*;
import team3176.robot.commands.superstructure.arm.*;
import team3176.robot.commands.superstructure.claw.*;
import team3176.robot.commands.superstructure.claw.ClawIdle;
import team3176.robot.commands.superstructure.intakecube.*;
import team3176.robot.commands.vision.*;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.RobotState;
import team3176.robot.subsystems.superstructure.Arm;
import team3176.robot.subsystems.superstructure.Claw;
import team3176.robot.subsystems.superstructure.IntakeCube;
import team3176.robot.subsystems.superstructure.IntakeCone;

import team3176.robot.subsystems.superstructure.Superstructure;
import team3176.robot.subsystems.vision.VisionDual;
import team3176.robot.subsystems.vision.VisionDualIOLime;

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

  private final Arm m_Arm;
  private final Controller m_Controller;
  private final Claw m_Claw;
  private final IntakeCube m_IntakeCube;
  private final IntakeCone m_IntakeCone;
  private PowerDistribution m_PDH; 


  // private final Compressor m_Compressor;
  private final Drivetrain m_Drivetrain;
  private final VisionDual m_Vision;
  private final Superstructure m_Superstructure;
  private SendableChooser<String> m_autonChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Arm = Arm.getInstance();
    m_Controller = Controller.getInstance();
    m_Claw = Claw.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    m_IntakeCube = IntakeCube.getInstance();
    m_IntakeCone = IntakeCone.getInstance();
    m_PDH = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kRev);

    m_Vision = VisionDual.getInstance();
    m_Superstructure = Superstructure.getInstance();
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin()));
    m_Arm.setDefaultCommand(m_Arm.armFineTune( () -> m_Controller.operator.getLeftY()));
    m_autonChooser = new SendableChooser<>();
    File paths = new File(Filesystem.getDeployDirectory(), "pathplanner");
    for (File f : paths.listFiles()) {
      if (!f.isDirectory()) {
        String s = f.getName().split("\\.", 0)[0];
        m_autonChooser.addOption(s, s);
      }
    }

  
    SmartDashboard.putData("Auton Choice", m_autonChooser);
    
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_Controller.getTransStick_Button1().whileTrue(m_Claw.scoreGamePiece());
    //m_Controller.getTransStick_Button1().onFalse(new InstantCommand(() -> m_Drivetrain.setTurbo(false), m_Drivetrain));
    m_Controller.getTransStick_Button2().whileTrue(new IntakeGroundCube());
    m_Controller.getTransStick_Button2().onFalse(new IntakeRetractSpinot().andThen(m_Superstructure.prepareCarry()));
    m_Controller.getTransStick_Button2().onFalse(m_Superstructure.prepareCarry());
    m_Controller.getTransStick_Button3().whileTrue(new SetColorWantState(3));
    m_Controller.getTransStick_Button3().whileTrue(m_Superstructure.groundCube());
    m_Controller.getTransStick_Button3().onFalse(new IntakeRetractSpinot());
    m_Controller.getTransStick_Button3().onFalse(m_Superstructure.prepareCarry());

    m_Controller.getTransStick_Button4().whileTrue(m_Superstructure.prepareScoreHigh());
    m_Controller.getTransStick_Button4().onFalse((m_Superstructure.prepareCarry()));
    m_Controller.getTransStick_Button5().onTrue(new InstantCommand(() -> m_Drivetrain.resetPoseToVision(),m_Drivetrain));
    m_Controller.getTransStick_Button10().whileTrue(new InstantCommand(() -> m_Drivetrain.setBrakeMode()).andThen(new SwerveDefense()));
     //m_Controller.getTransStick_Button10()
     //    .onFalse(new InstantCommand(() -> m_Drivetrain.setDriveMode(driveMode.DRIVE), m_Drivetrain));

    // m_Controller.getRotStick_Button2().whileTrue(new FlipField);
    m_Controller.getRotStick_Button1().whileTrue(new CubeChase(
      () -> m_Controller.getForward(),
      () -> m_Controller.getStrafe(),
      () -> m_Controller.getSpin())
    );
 
    m_Controller.getRotStick_Button2().whileTrue(new SpinLockDrive(
      () -> m_Controller.getForward(),
      () -> m_Controller.getStrafe())
    ); 
    

    m_Controller.getRotStick_Button3().whileTrue(new InstantCommand(() -> m_Drivetrain.setBrakeMode()).andThen(new SwerveDefense()));

    m_Controller.getRotStick_Button4().whileTrue(m_Superstructure.intakeCubeHumanPlayer());
    m_Controller.getRotStick_Button4().onFalse(m_Superstructure.prepareCarry());
    m_Controller.getTransStick_Button8()
        .whileTrue(new InstantCommand(() -> m_Drivetrain.resetFieldOrientation(), m_Drivetrain));

    double conveyorBumpTime = .1;  //In units of seconds
    m_Controller.operator.povUp().whileTrue(m_Superstructure.prepareScoreHigh());
    m_Controller.operator.povUp().onTrue(m_IntakeCube.bumpConveyor().withTimeout(conveyorBumpTime));
    m_Controller.operator.povRight().whileTrue(m_Superstructure.prepareCarry());
    m_Controller.operator.povRight().onTrue(m_IntakeCube.bumpConveyor().withTimeout(conveyorBumpTime));
    m_Controller.operator.povDown().whileTrue(m_Superstructure.prepareCatch());
    m_Controller.operator.povDown().onTrue(m_IntakeCube.bumpConveyor().withTimeout(conveyorBumpTime));
    m_Controller.operator.povLeft().whileTrue(m_Superstructure.prepareScoreMid());
    m_Controller.operator.povLeft().onTrue(m_IntakeCube.bumpConveyor().withTimeout(conveyorBumpTime));

    // m_Controller.operator.start().onTrue(new ToggleVisionLEDs());
    // m_Controller.operator.back().onTrue(new SwitchToNextVisionPipeline());

    m_Controller.operator.b().onTrue(new SetColorWantState(1));
    m_Controller.operator.b().whileTrue(m_Superstructure.intakeConeHumanPlayer());
    m_Controller.operator.b().onFalse(m_Superstructure.prepareCarry());
    
    m_Controller.operator.x().onTrue(new SetColorWantState(2));
    m_Controller.operator.x().whileTrue(m_Superstructure.intakeCubeHumanPlayer());
    m_Controller.operator.x().onFalse(m_Superstructure.prepareCarry());

    m_Controller.operator.a().onTrue(new SetColorWantState(3));
    m_Controller.operator.a().whileTrue(m_Superstructure.groundCube());
    m_Controller.operator.a().onFalse(new IntakeRetractSpinot());
    m_Controller.operator.a().onFalse(m_Superstructure.prepareCarry());

    m_Controller.operator.y().onTrue(new SetColorWantState(0));
    m_Controller.operator.y().whileTrue(m_Claw.scoreGamePiece());
    m_Controller.operator.y().onFalse(new ClawIdle());


    m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).onTrue(new SetColorWantState(3));
    m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).whileTrue(new IntakeGroundCube());
    m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).onFalse(new IntakeRetractSpinot());
    //m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).onFalse(m_Superstructure.prepareCarry());
    
    m_Controller.operator.leftBumper().and(m_Controller.operator.rightBumper()).whileTrue((new PoopCube()));
//    m_Controller.operator.leftBumper().and(m_Controller.operator.rightBumper()).onFalse(new InstantCommand( () -> m_IntakeCone.idle())); 
    

    
    m_Controller.operator.leftTrigger().onTrue(m_Arm.armSetPositionOnce(140).andThen(m_Arm.armFineTune( () -> m_Controller.operator.getLeftY())));
    //m_Controller.operator.leftBumper().onTrue(m_Arm.armSetPositionOnce(200).andThen(m_Arm.armFineTune( () -> m_Controller.operator.getLeftY())));
    //m_Controller.operator.leftBumper().onTrue(new ArmFollowTrajectory(SuperStructureConstants.ARM_MID_POS));
    //m_Controller.operator.start().whileTrue(new OldPoopCube());
    m_Controller.operator.start().whileTrue(new InstantCommand( () -> m_IntakeCone.spit()));
    m_Controller.operator.start().onFalse(new InstantCommand( () -> m_IntakeCone.idle()));
    //m_Controller.operator.start().onFalse(new IntakeRetractSpinot().andThen(m_Superstructure.prepareCarry()));
    m_Controller.operator.back().whileTrue(m_Superstructure.preparePoop());
    //m_Controller.operator.leftTrigger().whileTrue(new PoopCube()); 
    m_Controller.operator.rightTrigger().whileTrue(m_Superstructure.preparePoop());
  }

  public void setArmCoast() {
    m_Arm.setCoastMode();
  }
  
  public void setArmBrake() {
    m_Arm.setBrakeMode();
  }
  
  public void setThrustCoast() {
    m_Drivetrain.setCoastMode();
  }

  public void setThrustBrake() {
    m_Drivetrain.setBrakeMode();
  }

  public void clearCanFaults(){
    m_PDH.clearStickyFaults();
  }

  public void printCanFaults(){
    m_PDH.getStickyFaults();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String chosen = m_autonChooser.getSelected();
    //String chosen = "wall_cone_exit_balance";

    PathPlannerAuto PPSwerveauto = new PathPlannerAuto(chosen);
    return PPSwerveauto.getauto();
  }
}
