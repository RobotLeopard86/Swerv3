package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FFwdGains;
import frc.robot.Constants.PIDGains;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		// Drive motor data
		public double driveMotorPositionRad;
		public double driveMotorVelocityRadPerSec;
		public double driveMotorAppliedVolts;
		public double driveMotorCurrentAmpsSupply;
		public boolean driveMotorConnected = false;

		// Turn motor data
		public Rotation2d turnMotorPosition = Rotation2d.kZero;
		public Rotation2d turnMotorAbsPosition = Rotation2d.kZero;
		public double turnMotorVelocityRadPerSec;
		public double turnMotorAppliedVolts;
		public double turnMotorCurrentAmpsSupply;
		public boolean turnMotorConnected = false;
		public boolean turnAbsEncoderConnected = false;
	}

	// Update loggable inputs from the underlying hardware
	default void updateInputs(ModuleIOInputs inputs) {
	}

	// Set the voltage of the drive motor
	default void setDriveMotorVoltage(double voltage) {
	}

	// Set the voltage of the turn motor
	default void setTurnMotorVoltage(double voltage) {
	}

	// Set drive motor velocity
	default void setDriveMotorVelocity(double velocityRadPerSec) {
	}

	// Set position of the turn motor
	default void setTurnMotorPosition(double position) {
	}

	// Set drive motor PID gains
	default void setDriveMotorPIDGains(PIDGains gains) {
	}

	// Set turn motor PID gains
	default void setTurnMotorPIDGains(PIDGains gains) {
	}

	// Set drive motor feedforward gains
	default void setDriveMotorFFwdGains(FFwdGains gains) {
	}

	// Set turn motor feedforward gains
	default void setTurnMotorFFwdGains(FFwdGains gains) {
	}

	// Enable/disable brake on drive motor
	default void setDriveMotorBrake(boolean brake) {
	}

	// Enable/disable brake on turn motor
	default void setTurnMotorBrake(boolean brake) {
	}

	// Stop the motor
	default void stop() {
	}
}
