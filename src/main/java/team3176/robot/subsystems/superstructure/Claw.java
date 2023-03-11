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

import team3176.robot.subsystems.superstructure.ClawIO;
import team3176.robot.subsystems.superstructure.ClawIO.ClawHardware;
import team3176.robot.subsystems.superstructure.ClawIO.ClawIOInputs;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    private CANSparkMax claw;
    private DigitalInput linebreakOne;
    private DigitalInput linebreakTwo;
    private static Claw instance;
    private final ClawIO io;
    private final ClawIOInputs inputs = new ClawIOInputs();
    public final ClawHardware hardware = new ClawHardware();
    public GamePiece currentGamePiece = GamePiece.CONE;
    private Claw(ClawIO io) {
        this.io = io;
        claw = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
        linebreakOne = new DigitalInput(9);
        linebreakTwo = new DigitalInput(7);
    }
    public void setClawMotor(double percent, double amps) {
        // claw.set(percent);
        //claw.setSmartCurrentLimit(SuperStructureConstants.CLAW_AMPS);
        setVelocity(percent);
        setAmps(SuperStructureConstants.CLAW_AMPS);
        hardware.setVelocity(inputs.velocity);
        hardware.setAmps((int) inputs.currentAmps);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", hardware.getCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", hardware.getTemperature());
    }

    //states now implemented as functions
    
    public void intake() {
        //System.out.println("m_Claw.intake()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
         else if(currentGamePiece == GamePiece.CONE) {
             setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
    }
    
    public void hold() {
        //System.out.println("m_Claw.hold()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
            //idle(); 
        } else {
            setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER * SuperStructureConstants.CLAW_HOLD_CONE_FACTOR,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
        }
    }

    public void score() {
        //System.out.println("m_Claw.score()");
        if(currentGamePiece == GamePiece.CUBE) {
            setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
        else if(currentGamePiece == GamePiece.CONE) {
            setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
    }
    public void idle() {
        //System.out.println("m_Claw.idle()");
        setClawMotor(0, 0);
    }

    public void setCurrentGamePiece(GamePiece piece) {
        //System.out.println("m_Claw.setCurrentGamePiece() to " + piece);
        this.currentGamePiece = piece;
    }

    public boolean isClawHolding()
    {
        return hardware.getLinebreakOne() || hardware.getLinebreakTwo();
    }

    public boolean isEmpty() {
        return hardware.getLinebreakOne() || hardware.getLinebreakTwo();
    }

    public static Claw getInstance()
    {
        if (instance == null ) {
            instance = new Claw(new ClawIO() {});
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
                    .until(() -> this.isEmpty())
                    .andThen(new WaitCommand(0.5))
                    .andThen(this.runOnce(()->idle())).withTimeout(2.0).finallyDo((b)->idle());
    }

    //more examples of command composition and why its awesome!!
    // public Command intakeCone() {
    //     return this.intakeGamePiece(GamePiece.CONE).until(this::getLinebreakTwo);
    // }
    // public Command intakeCube() {
    //     return this.intakeGamePiece(GamePiece.CUBE).until(this::getLinebreakOne);
    // }

    public Command determineGamePiece() {
        return this.runOnce( () -> {
            if(this.hardware.getLinebreakOne()) {
                this.currentGamePiece = GamePiece.CONE;
                hold();
            } else if(this.hardware.getLinebreakTwo()) {
                this.currentGamePiece = GamePiece.CUBE;
                hold();
            }
        });
    }



    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
    Logger.getInstance().recordOutput("Claw/Velocity", getVelocity());
    Logger.getInstance().recordOutput("Claw/LinebreakOne", getIsLinebreakOne());
    Logger.getInstance().recordOutput("Claw/LinebreakTwo", getIsLinebreakTwo());
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("linebreakOne",linebreakOne.get());
    SmartDashboard.putBoolean("linebreakTwo",linebreakTwo.get());
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

   public double getVelocity()
   {
    return inputs.velocity;
   }

   public boolean getIsLinebreakOne()
   {
    return inputs.isLinebreakOne;
   }

   public boolean getIsLinebreakTwo()
   {
    return inputs.isLinebreakTwo;
   }

   public void runVoltage(double volts)
   {
    io.setVoltage(volts);
   }

   public void setAmps(double amps)
   {
    io.setAmps(amps);
   }

   public void setVelocity(double velocity)
   {
    io.setVelocity(velocity);
   }
}
