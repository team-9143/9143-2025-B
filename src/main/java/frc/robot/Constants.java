package frc.robot;

/*
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class KitBotConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_FIRST_EJECT_VALUE = -0.2;
    public static final double ROLLER_STACKED_EJECT_VALUE = -0.3;
    public static final double ROLLER_REALIGN_VALUE = 0.1;
    public static final double ROLLER_JOG_VALUE = -0.02;
  }

  public final class AlLowConstants {
		// Motor IDs
		public static final int ALLOW_PIVOT_MOTOR_ID = 61; // CAN ID for the pivot motor
		public static final int ALLOW_ROLLER_MOTOR_ID = 62; // CAN ID for the roller motor

		// Motor Inversion
		public static final boolean ALLOW_PIVOT_MOTOR_INVERTED = false; // Set to true if the pivot motor is inverted
		public static final boolean ALLOW_ROLLER_MOTOR_INVERTED = false; // Set to true if the roller motor is inverted

		// Current Limits
		public static final int ALLOW_PIVOT_CURRENT_LIMIT = 30; // Current limit for the pivot motor (in amps)
		public static final int ALLOW_ROLLER_CURRENT_LIMIT = 20; // Current limit for the roller motor (in amps)

		// Encoder Conversion Factors
		public static final double ALLOW_PIVOT_POSITION_CONVERSION = 13.334; // Convert encoder ticks to degrees (placeholder value)
		public static final double ALLOW_PIVOT_VELOCITY_CONVERSION = 13.334 / 60.0; // Convert encoder ticks to degrees per second (placeholder value)

		// PID Constants for Pivot Motor
		public static final double ALLOW_PIVOT_kP = 0.01; // Proportional gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kI = 0.001; // Integral gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kD = 0.0; // Derivative gain for the pivot motor's PID controller
		public static final double ALLOW_PIVOT_kF = 0.0; // Feedforward gain for gravity compensation

		// Pivot Angle Limits
		public static final double ALLOW_PIVOT_MIN_ANGLE = 0.0; // Minimum allowed angle for the pivot (in degrees)
		public static final double ALLOW_PIVOT_MAX_ANGLE = 60.0; // Maximum allowed angle for the pivot (in degrees)

		// Allowed Error
		public static final double ALLOW_PIVOT_ALLOWED_ERROR = 5.0; // Allowed error threshold for the pivot to be "at target" (in degrees)

		// Manual Control Parameters
		public static final double ALLOW_MANUAL_CONTROL_DEADBAND = 0.2; // Deadband for manual pivot control
		public static final double ALLOW_MANUAL_SPEED_LIMIT = 0.15; // Speed limit for manual pivot control

		// Pivot Preset Angles
		public enum PivotPresetAngles {
			BASE(0.0),
			INTAKE(45.0),
      		HOLD(15.0);

			private final double angle;

			PivotPresetAngles(double angle) {
				this.angle = angle;
			}

			public double getAngle() {
				return angle;
			}
		}
	}
}