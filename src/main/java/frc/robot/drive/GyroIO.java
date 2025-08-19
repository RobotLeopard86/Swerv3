package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
	@AutoLog
	public static class GyroIOInputs {
		public Rotation2d yaw = Rotation2d.kZero;
		public double yawVelocityRadPerSec = 0.0f;
		public boolean connected = false;
	}

	// Update loggable inputs from the underlying hardware
	default void updateInputs(GyroIOInputs inputs) {
	}

	// Reset the gyro heading to zero
	default void reset() {
	}
}
