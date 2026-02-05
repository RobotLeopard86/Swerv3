package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class Constants {
	// Xbox controller port
	public static final int XBOX_PORT = 0;

	// Joystick control deadbamd
	public static final double JOYSTICK_DEADBAND = 0.15;

	// Type of robot we're running on
	public enum RobotType {
		SIM,
		PRESEASON_2026
	}

	public static final RobotType ROBOT_TYPE = (RobotBase.isReal() ? RobotType.PRESEASON_2026 : RobotType.SIM);

	// Runtime environment
	public enum Environment {
		REALITY,
		SIM,
		REPLAY
	}

	public static final Environment ENV = switch(ROBOT_TYPE) {
		case SIM -> Environment.SIM;
		case PRESEASON_2026 -> Environment.REALITY;
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
		// These values (for sim bot) were copied from DriveConstants.java in
		// redshiftrobotics/reefscape-2025
		case SIM -> new DriveConfig(new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 5.05968, 14.5);

		case PRESEASON_2026 -> new DriveConfig(new Translation2d(
				TunerConstants.FrontLeft.LocationX - TunerConstants.BackRight.LocationX,
				TunerConstants.FrontLeft.LocationY - TunerConstants.BackRight.LocationY),
				new Translation2d(
						TunerConstants.FrontLeft.LocationX - TunerConstants.BackRight.LocationX,
						TunerConstants.FrontLeft.LocationY - TunerConstants.BackRight.LocationY)
								.plus(new Translation2d(6.750, 6.750).times(2)),
				TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 22.0);
	};

	// Module positions
	public static final double CENTER_X = DRIVE_CFG.frameDiagonal.getX() / 2;
	public static final double CENTER_Y = DRIVE_CFG.frameDiagonal.getY() / 2;
	public static final Translation2d MODULE_FL_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, CENTER_Y);
	public static final Translation2d MODULE_FR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, -CENTER_Y);
	public static final Translation2d MODULE_BL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, CENTER_Y);
	public static final Translation2d MODULE_BR_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, -CENTER_Y);

	// Wheel radius
	public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

	// Swerve module configuration
	public record SwerveModuleConfig(
			int driveMotorID, int turnMotorID, int encoderID, Rotation2d encoderOffset, boolean turnMotorInverted) {
	}

	public static final SwerveModuleConfig MODULE_FL_CFG = switch(ROBOT_TYPE) {
		case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
		// DO NOT RELY ON THIS FOR PRESEASON_2026, USE THE PER-MODULE CONSTANTS VALUES
		// FROM TunerConstants!!!
		case PRESEASON_2026 -> new SwerveModuleConfig(TunerConstants.FrontLeft.DriveMotorId,
				TunerConstants.FrontLeft.SteerMotorId, TunerConstants.FrontLeft.EncoderId,
				Rotation2d.fromRotations(TunerConstants.FrontLeft.EncoderOffset),
				TunerConstants.FrontLeft.SteerMotorInverted);
	};
	public static final SwerveModuleConfig MODULE_FR_CFG = switch(ROBOT_TYPE) {
		case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
		// DO NOT RELY ON THIS FOR PRESEASON_2026, USE THE PER-MODULE CONSTANTS VALUES
		// FROM TunerConstants!!!
		case PRESEASON_2026 -> new SwerveModuleConfig(TunerConstants.FrontRight.DriveMotorId,
				TunerConstants.FrontRight.SteerMotorId, TunerConstants.FrontRight.EncoderId,
				Rotation2d.fromRotations(TunerConstants.FrontRight.EncoderOffset),
				TunerConstants.FrontRight.SteerMotorInverted);
	};
	public static final SwerveModuleConfig MODULE_BL_CFG = switch(ROBOT_TYPE) {
		case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
		// DO NOT RELY ON THIS FOR PRESEASON_2026, USE THE PER-MODULE CONSTANTS VALUES
		// FROM TunerConstants!!!
		case PRESEASON_2026 -> new SwerveModuleConfig(TunerConstants.BackLeft.DriveMotorId,
				TunerConstants.BackLeft.SteerMotorId, TunerConstants.BackLeft.EncoderId,
				Rotation2d.fromRotations(TunerConstants.BackLeft.EncoderOffset),
				TunerConstants.BackLeft.SteerMotorInverted);
	};
	public static final SwerveModuleConfig MODULE_BR_CFG = switch(ROBOT_TYPE) {
		case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
		// DO NOT RELY ON THIS FOR PRESEASON_2026, USE THE PER-MODULE CONSTANTS VALUES
		// FROM TunerConstants!!!
		case PRESEASON_2026 -> new SwerveModuleConfig(TunerConstants.BackRight.DriveMotorId,
				TunerConstants.BackRight.SteerMotorId, TunerConstants.BackRight.EncoderId,
				Rotation2d.fromRotations(TunerConstants.BackRight.EncoderOffset),
				TunerConstants.BackRight.SteerMotorInverted);
	};

	// Gyro ID
	public static final int GYRO_ID = switch(ROBOT_TYPE) {
		case SIM -> -1;
		case PRESEASON_2026 -> TunerConstants.DrivetrainConstants.Pigeon2Id;
	};

	// Controller gains
	public record FFwdGains(double kS, double kV, double kA) {
	}

	public record PIDGains(double kP, double kI, double kD) {
	}

	// Current gain settings copied from ModuleConstants.java,
	// redshiftrobotics/reefscape-2025 & redshiftrobotics/preseason-2026
	public static final FFwdGains DRIVE_FEEDFWD_GAINS = switch(ROBOT_TYPE) {
		case SIM -> new FFwdGains(0, 0, 0);
		case PRESEASON_2026 -> new FFwdGains(2.00544, 1.05719, 0);
	};

	public static final PIDGains DRIVE_PID_GAINS = switch(ROBOT_TYPE) {
		case SIM -> new PIDGains(1.3, 0, 0);
		case PRESEASON_2026 -> new PIDGains(20, 0, 0);
	};

	public static final FFwdGains TURN_FEEDFWD_GAINS = switch(ROBOT_TYPE) {
		case SIM -> new FFwdGains(0, 0, 0);
		case PRESEASON_2026 -> new FFwdGains(0.2, 0, 0);
	};

	public static final PIDGains TURN_PID_GAINS = switch(ROBOT_TYPE) {
		case SIM -> new PIDGains(10.0, 0, 0);
		case PRESEASON_2026 -> new PIDGains(1400, 0, 15);
	};

	// Reductions were copied from ModuleConstants.java in
	// redshiftrobotics/reefscape-2025!
	// Originally sourced from:
	// https://www.swervedrivespecialties.com/products/mk4i-swerve-module
	public enum Mk4iReductions {
		// Note: Mk4i turn motors are inverted!
		L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
		L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
		L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
		TURN((150.0 / 7.0));

		public final double reduction;

		Mk4iReductions(double reduction) {
			this.reduction = reduction;
		}
	}

	// Reductions were copied from ModuleConstants.java in
	// redshiftrobotics/preseason-2026!
	// Originally sourced from:
	// https://www.swervedrivespecialties.com/products/mk5n-swerve-module
	public enum Mk5nReductions {
		L1(12.0),
		L2(14.0),
		L3(16.0);

		public static final double TURN_REDUCTION = (287.0 / 11.0);

		public final double reduction;

		Mk5nReductions(double adjustableGearTeeth) {
			this.reduction = (54.0 / adjustableGearTeeth) * (25.0 / 32.0) * (30.0 / 15.0);
		}
	}

	// Reductions
	public static final double DRIVE_REDUCTION = switch(ROBOT_TYPE) {
		case SIM -> Mk4iReductions.L3.reduction;
		case PRESEASON_2026 -> Mk5nReductions.L3.reduction;
	};

	public static final double TURN_REDUCTION = switch(ROBOT_TYPE) {
		case SIM -> Mk4iReductions.TURN.reduction;
		case PRESEASON_2026 -> Mk5nReductions.TURN_REDUCTION;
	};

	// Drive odometry polling frequency
	public static final double DRIVE_ODOMETRY_FREQUENCY_HZ = switch(ROBOT_TYPE) {
		case SIM -> 50.0;
		case PRESEASON_2026 -> (TunerConstants.kCANBus.isNetworkFD() ? 250.0
				: 100.0);
		default -> 100.0;
	};

	// Drivetrain device disconnect warning thresholds
	// Basically, if the device is still disconnected after this time we know
	// something is up
	public static final double DRIVE_MOTOR_DISCONNECT_WARNING_THRESHOLD_SECONDS = 0.5;
	public static final double DRIVE_ENCODER_DISCONNECT_WARNING_THRESHOLD_SECONDS = 0.2;
}
