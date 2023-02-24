package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.claw.ClawInhaleCone;
import team3176.robot.commands.claw.ClawInhaleCube;
import team3176.robot.commands.intake.IntakeExtendSpin;
import team3176.robot.commands.intake.IntakeRetractSpinot;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    private Arm m_Arm;
    private Claw m_Claw;
    private Intake m_Intake;
    public Superstructure() {
        m_Arm = Arm.getInstance();
        m_Claw = Claw.getInstance();
        m_Intake = Intake.getInstance();
    }
    public static Superstructure getInstance() {
        if (instance == null){instance = new Superstructure();}
        return instance;
    }
    
    

    public static enum GamePiece {CUBE, CONE, NONE};
    

    public Command groundCube() {
        return new ParallelCommandGroup(m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CATCH_POS),
                                        new IntakeExtendSpin(), 
                                        new ClawInhaleCube())
                    .andThen(new IntakeRetractSpinot());
    }

    public Command poopCube() {
        return new ParallelCommandGroup(new IntakeExtendFreeSpin())
                    .andThen(this.preparePoop())
                    .andThen(m_Claw.scoreGamePiece())
                    .andThen(this.prepareCarry())
                    .andThen(new IntakeRetractSpinot());
            //May need to add Wait Cmds in the above logic
    }

    public Command intakeCubeHumanPlayer() {
        return new ParallelCommandGroup(new ClawInhaleCube(), m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS))
        .andThen(m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS));
    }

    public Command intakeConeHumanPlayer() {
        return new ParallelCommandGroup(new ClawInhaleCone(), m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS))
        .andThen(m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS));
    }
    
    public Command preparePoop() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_ZERO_POS);
    }
    public Command prepareCarry() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS);
    }
    public Command prepareCatch() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_CATCH_POS);
    }
    public Command prepareScoreMid() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_MID_POS);
    }
    public Command prepareScoreHigh() {
        return m_Arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS);
    }
    
}
