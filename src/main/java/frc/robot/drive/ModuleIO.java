package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PIDGains;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		// Drive motor data
		double driveMotorPositionRad;
		double driveMotorVelocityRadPerSec;
		double driveMotorAppliedVolts;
		double driveMotorCurrentAmpsSupply;

		// Turn motor data
		Rotation2d turnMotorPosition = Rotation2d.kZero;
		Rotation2d turnMotorAbsPosition = Rotation2d.kZero;
		double turnMotorVelocityRadPerSec;
		double turnMotorAppliedVolts;
		double turnMotorCurrentAmpsSupply;
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

	// Set drive motor velocity using the provided feedforward voltage
	default void setDriveMotorVelocity(double velocityRadPerSec, double feedforwardVoltage) {
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
