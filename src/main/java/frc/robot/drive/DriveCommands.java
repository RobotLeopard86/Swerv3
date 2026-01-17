package frc.robot.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommands {
	public static Command teleopDrive(Drivetrain drive, Supplier<Translation2d> translation, DoubleSupplier omega) {
		return drive.run(() -> {
			// Create chassis speeds object
			Translation2d translate = translation.get();
			ChassisSpeeds speeds = new ChassisSpeeds(translate.getX(), translate.getY(), omega.getAsDouble());

			// Apply speeds to drivetrain
			drive.setTargetRobotSpeeds(speeds);
		}).finallyDo(drive::stop);
	}
}
