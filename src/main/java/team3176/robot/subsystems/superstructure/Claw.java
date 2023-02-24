package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;

public class Claw extends SubsystemBase {
    private CANSparkMax claw;
    private DigitalInput linebreakOne;
    private DigitalInput linebreakTwo;
    private static Claw instance;
    public GamePiece currentGamePiece = GamePiece.NONE;
    private Claw() {
        claw = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
        linebreakOne = new DigitalInput(9);
        linebreakTwo = new DigitalInput(7);
    }
    public void setClawMotor(double percent, double amps) {
        claw.set(percent);
        claw.setSmartCurrentLimit(SuperStructureConstants.CLAW_AMPS);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", claw.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", claw.getMotorTemperature());
    }

    //states now implemented as functions
    
    public void intake() {
        System.out.println("m_Claw.intake()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
         else if(currentGamePiece == GamePiece.CONE) {
             setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
    }
    
    public void hold() {
        System.out.println("m_Claw.hold()");
        if(currentGamePiece == GamePiece.CUBE) {
            //setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
            idle(); 
        } else {
            setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
        }
    }

    public void score() {
        System.out.println("m_Claw.score()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
        else if(currentGamePiece == GamePiece.CONE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
    }
    public void idle() {
        System.out.println("m_Claw.idle()");
        setClawMotor(0, 0);
    }

    public void setCurrentGamePiece(GamePiece piece) {
        System.out.println("m_Claw.setCurrentGamePiece() to " + piece);
        this.currentGamePiece = piece;
    }

    public boolean getLinebreakOne()
    {
        return linebreakOne.get();
    }
    
    public boolean getLinebreakTwo()
    {
        return linebreakTwo.get();
    }
    public boolean isEmpty() {
        return getLinebreakOne() || getLinebreakTwo();
    }

    public static Claw getInstance()
    {
        if (instance == null ) {
            instance = new Claw();
          }
        return instance;
    }

    /**
     * to be called with a whileTrue trigger binding
     */
    public CommandBase intakeGamePiece(GamePiece piece) {
        return this.startEnd(() ->  {this.currentGamePiece = piece; intake();},() -> hold());
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue or onTrue trigger binding
     */
    public Command scoreGamePiece() {
        return this.run(() ->  {score(); this.currentGamePiece = GamePiece.NONE;})
                    .until(() -> this.isEmpty()).
                    andThen(new WaitCommand(0.5))
                    .andThen(this.runOnce(()->idle()));
    }

    //more examples of command composition and why its awesome!!
    public Command intakeCone() {
        return this.intakeGamePiece(GamePiece.CONE).until(this::getLinebreakTwo);
    }
    public Command intakeCube() {
        return this.intakeGamePiece(GamePiece.CUBE).until(this::getLinebreakOne);
    }



    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("linebreakOne",linebreakOne.get());
    SmartDashboard.putBoolean("linebreakTwo",linebreakTwo.get());
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }
}
