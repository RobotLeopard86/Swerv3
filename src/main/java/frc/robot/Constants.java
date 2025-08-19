package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
	// Xbox controller port
	public static final int XBOX_PORT = 0;

	// Joystick control deadbamd
	public static final double JOYSTICK_DEADBAND = 0.15;

	// Type of robot we're running on
	public enum RobotType {
		SIM
	}

	public static final RobotType ROBOT_TYPE = RobotType.SIM;

	// Runtime environment
	public enum Environment {
		REALITY,
		SIM,
		REPLAY
	}

	public static final Environment ENV = switch(ROBOT_TYPE) {
	case SIM -> Environment.SIM;
	default -> RobotBase.isReal() ? Environment.REALITY : Environment.REPLAY;
	};

	// Command scheduler loop period
	public static final double LOOP_PERIOD = 0.02;

	// Drive configuration
	// frameDiagonal is the diagonal length between the corners of the metal frame
	// bumperDiagnol is the same but between the corners of the bumpers
	public record DriveConfig(
			Translation2d frameDiagonal, Translation2d bumperDiagonal,
			double maxLinearVelocity, double maxLinearAcceleration) {

		public double getBaseRadius() {
			return frameDiagonal.getNorm() / 2;
		}

		public double maxAngularVelocity() {
			return maxLinearVelocity / getBaseRadius();
		}

		public double maxAngularAcceleration() {
			return maxLinearAcceleration / getBaseRadius();
		}
	}

	public static final DriveConfig DRIVE_CFG = switch(ROBOT_TYPE) {
	// These values were copied from DriveConstants.java in
	// redshiftrobotics/reefscape-2025
	case SIM -> new DriveConfig(new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);
	};

	// Module positions
	public static final double CENTER_X = DRIVE_CFG.frameDiagonal.getX() / 2;
	public static final double CENTER_Y = DRIVE_CFG.frameDiagonal.getY() / 2;
	public static final Translation2d MODULE_FL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, CENTER_Y);
	public static final Translation2d MODULE_FR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, CENTER_Y);
	public static final Translation2d MODULE_BL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, -CENTER_Y);
	public static final Translation2d MODULE_BR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, -CENTER_Y);

	// Wheel radius
	public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

	// Swerve module configuration
	public record SwerveModuleConfig(
			int driveMotorID, int turnMotorID, int absEncoder, Rotation2d absEncoderOffset, boolean turnMotorInverted) {
	}

	public static final SwerveModuleConfig MODULE_CFG = switch(ROBOT_TYPE) {
	case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
	};

	// Controller gains
	public record FeedforwardGains(double kS, double kV, double kA) {
	}

	public record PIDGains(double kP, double kI, double kD) {
	}

	// Current gain settings copied from ModuleConstants.java,
	// redshiftrobotics/reefscape-2025
	public static final FeedforwardGains DRIVE_FEEDFWD_GAINS = new FeedforwardGains(0, 0, 0);
	public static final PIDGains DRIVE_PID_GAINS = new PIDGains(1.3, 0, 0);
	public static final PIDGains TURN_PID_GAINS = new PIDGains(10.0, 0, 0);

	// Reductions were copied from ModuleConstants.java in
	// redshiftrobotics/reefscape-2025!
	// Originally sourced from:
	// https://www.swervedrivespecialties.com/products/mk4i-swerve-module
	private enum Mk4iReductions {
		// Note: Mk4i turn motors are inverted!
		L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
		L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
		L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
		TURN((150.0 / 7.0));

		final double reduction;

		Mk4iReductions(double reduction) {
			this.reduction = reduction;
		}
	}

	// Reductions
	public static final double DRIVE_REDUCTION = switch(ROBOT_TYPE) {
	case SIM -> Mk4iReductions.L3.reduction;
	};
	public static final double TURN_REDUCTION = switch(ROBOT_TYPE) {
	case SIM -> Mk4iReductions.TURN.reduction;
	};
}
