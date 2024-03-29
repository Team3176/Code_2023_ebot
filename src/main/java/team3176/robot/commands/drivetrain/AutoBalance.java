package team3176.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;

public class AutoBalance extends CommandBase {
    private Drivetrain m_Drivetrain;
    private boolean isDone = false;
    private int num_balanced = 0;
    public AutoBalance() {
        m_Drivetrain = Drivetrain.getInstance();
        addRequirements(m_Drivetrain);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_Drivetrain.setBrakeMode();

    }
    @Override
    public void execute() {
        //double Kp = 0.1;

        //Bang Bang controller! 
        double forward = 0.0;
        double deadbandDegrees = 8;
        SmartDashboard.putNumber("pitch", m_Drivetrain.getChassisPitch());
        // if(m_Drivetrain.getChassisPitch() > 0 + deadbandDegrees) {
        //     forward = 0.37 * Math.pow(.96,num_balanced);
        // } else if(m_Drivetrain.getChassisPitch() < 0 - deadbandDegrees) {
        //     forward = -0.37 * Math.pow(.96,num_balanced);
        // } else if(Math.abs(m_Drivetrain.getChassisPitch()) < 2){
        //     num_balanced ++;
        // }
        //P loop option
        if(Math.abs(m_Drivetrain.getChassisPitch()) > 0 + deadbandDegrees) {
            forward = 0.03 * m_Drivetrain.getChassisPitch();
            forward = MathUtil.clamp(forward, -0.5, 0.5);
        }
        else {
            forward = 0.0;
        }

        m_Drivetrain.drive(forward, 0, 0, Drivetrain.coordType.ROBOT_CENTRIC);
    }
    @Override
    public void end(boolean interrupted) {
        m_Drivetrain.setDriveMode(driveMode.DEFENSE);
    }
    @Override
    public boolean isFinished() {
        return Timer.getMatchTime() < 0.5;
    }

}
