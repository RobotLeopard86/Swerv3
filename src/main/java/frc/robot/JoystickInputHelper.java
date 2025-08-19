package frc.robot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class JoystickInputHelper {
	private DoubleSupplier xSupplier, ySupplier, yAngleSupplier;

	public JoystickInputHelper(DoubleSupplier xs, DoubleSupplier ys, DoubleSupplier yas) {
		xSupplier = xs;
		ySupplier = ys;
		yAngleSupplier = yas; // yas queen
	}

	public static double applyDeadbanding(double input) {
		return MathUtil.applyDeadband(input, Constants.JOYSTICK_DEADBAND);
	}

	public static boolean isDeadbandRange(double input) {
		return Math.abs(input) < Constants.JOYSTICK_DEADBAND;
	}

	// These calculations are mostly copied from JoystickInputController.java in
	// redshiftrobotics/reefscape-2025

	// Calculate robot translation in meters per second
	@AutoLogOutput(key = "TeleInput/TranslationMetersPerSecond")
	public Translation2d getTranslation() {
		// No effect in autonomous mode
		if(DriverStation.isAutonomous())
			return Translation2d.kZero;

		// Get current inputs
		Translation2d trans = new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

		// Deadbanding
		double magnitude = applyDeadbanding(trans.getNorm());
		if(magnitude <= 0)
			return Translation2d.kZero;

		// Calculate new translation
		// We apply ramp-up and fine control adjustments by squaring magnitude
		double magnitudeSquared = Math.pow(magnitude, 2);
		Translation2d trans2 = new Pose2d(Translation2d.kZero, trans.getAngle())
				.transformBy(new Transform2d(magnitudeSquared, 0.0, Rotation2d.kZero)).getTranslation();

		// Return final translation accounting for drive max speed
		return trans2.times(Constants.DRIVE_CFG.maxLinearVelocity());
	}

	// Calculate omega in radians per second
	@AutoLogOutput(key = "TeleInput/OmegaRadPerSec")
	public double getOmega() {
		// No effect in autonomous mode
		if(DriverStation.isAutonomous())
			return 0.0;

		// Deadbanding
		double omega = applyDeadbanding(yAngleSupplier.getAsDouble());

		// Square omega to match driving
		double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

		// Return omega accounting for max angular speed
		return omegaSquared * Constants.DRIVE_CFG.maxAngularVelocity();
	}
}
