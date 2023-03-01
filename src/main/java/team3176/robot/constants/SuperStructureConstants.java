package team3176.robot.constants;

public class SuperStructureConstants {
  
    /**
   * How many amps the arm motor can use.
   */
  public static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  public static final double ARM_OUTPUT_POWER = 0.2;

  /**
   * How many amps the claw can use while picking up
   */
  public static final int CLAW_CURRENT_LIMIT_A = 25;
  public static final int CLAW_AMPS = 25;

  /**
   * How many amps the claw can use while holding
   */
  public static final int CLAW_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for inhaling
   */
  public static final double CLAW_OUTPUT_POWER_CUBE = .6;
  public static final double CLAW_OUTPUT_POWER_CONE = 1.0;

  /**
   * Percent output for holding
   */
  public static final double CLAW_HOLD_POWER = 0.14;
  public static final double CLAW_HOLD_CONE_FACTOR = 2;

  public static final int ARM_ENCODER_OFFSET = 0; 

  public static final double ARM_kP = 0.02;
  public static final double ARM_kI = 0;
  public static final double ARM_kD = 0;
  public static final double ARM_kg = 0.2;
  public static final double ARM_TOLERANCE = 3;

  public static final double ARM_ZERO_POS = 6.0;
  public static final double ARM_CARRY_POS = 38 + ARM_ZERO_POS;
  public static final double ARM_CATCH_POS = 60 + ARM_ZERO_POS;
  public static final double ARM_MID_POS = 120 + ARM_ZERO_POS;
  public static final double ARM_HIGH_POS = 175 + ARM_ZERO_POS;
}
