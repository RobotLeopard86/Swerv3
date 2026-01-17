package frc.robot.drive;

import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	// Swerve modules
	private SwerveModule[] modules; // FL, FR, BL, BR order

	// Kinematics & odometry
	private SwerveDriveKinematics kinematics;
	private SwerveDrivePoseEstimator poseEstimator;

	// Current robot pose and speeds
	private Pose2d pose = new Pose2d();
	private ChassisSpeeds speeds = new ChassisSpeeds();
	private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

	// Gyro
	private GyroIO gyro;
	private GyroIOInputsAutoLogged gyroInputs;

	// Data for yaw calculation without gyro
	private SwerveModulePosition[] lastModulePositions;
	private Rotation2d yaw = Rotation2d.kZero;

	public Drivetrain(ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br, GyroIO gyro) {
		// Set modules
		modules = new SwerveModule[] {
				new SwerveModule(fl, Constants.MODULE_FL_DISTANCE_FROM_CENTER, "FL"),
				new SwerveModule(fr, Constants.MODULE_FR_DISTANCE_FROM_CENTER, "FR"),
				new SwerveModule(bl, Constants.MODULE_BL_DISTANCE_FROM_CENTER, "BL"),
				new SwerveModule(br, Constants.MODULE_BR_DISTANCE_FROM_CENTER, "BR"),
		};

		// Configure kinematics and odometry
		kinematics = new SwerveDriveKinematics(
				Arrays.stream(modules).map(SwerveModule::getDistanceFromCenter).toArray(Translation2d[]::new));
		lastModulePositions = Arrays.stream(modules).map(SwerveModule::getPosition)
				.toArray(SwerveModulePosition[]::new);
		poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, lastModulePositions, new Pose2d());

		// Set gyro
		this.gyro = gyro;
		gyroInputs = new GyroIOInputsAutoLogged();
	}

	void setTargetRobotSpeeds(ChassisSpeeds speeds) {
		// Make the speeds discrete to account for them only changing every loop period
		targetSpeeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD);
		this.speeds = targetSpeeds;

		// Calculate appropriate wheel speeds
		setTargetWheelSpeeds(kinematics.toSwerveModuleStates(this.speeds));
	}

	ChassisSpeeds getTargetRobotSpeeds() {
		return targetSpeeds;
	}

	ChassisSpeeds getMeasuredRobotSpeeds() {
		return speeds;
	}

	void setTargetWheelSpeeds(SwerveModuleState[] speeds) {
		// Adjust speeds to account for motor max speeds
		SwerveDriveKinematics.desaturateWheelSpeeds(speeds, Constants.DRIVE_CFG.maxLinearVelocity());

		// Apply speeds to modules
		for(int i = 0; i < modules.length; ++i) {
			modules[i].setTargetState(speeds[i]);
		}
	}

	SwerveModuleState[] getTargetWheelSpeeds() {
		return Arrays.stream(modules).map(SwerveModule::getTargetState).toArray(SwerveModuleState[]::new);
	}

	SwerveModuleState[] getMeasuredWheelSpeeds() {
		return Arrays.stream(modules).map(SwerveModule::getMeasuredState).toArray(SwerveModuleState[]::new);
	}

	@AutoLogOutput(key = "Pose")
	Pose2d getPose() {
		return pose;
	}

	void setPose(Pose2d pose) {
		// Reset the pose estimator
		poseEstimator.resetPosition(yaw,
				Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), pose);
	}

	public void stop() {
		setTargetRobotSpeeds(new ChassisSpeeds());
	}

	@AutoLogOutput(key = "Yaw")
	Rotation2d getYaw() {
		return yaw;
	}

	@Override
	public void periodic() {
		// Update inputs
		gyro.updateInputs(gyroInputs);
		Arrays.stream(modules).forEach(SwerveModule::updateInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);

		// Halt movement if disabled
		if(DriverStation.isDisabled())
			stop();

		// Log data to AdvantageKit
		Logger.recordOutput("Drive/WheelSpeeds/Target", getTargetWheelSpeeds());
		Logger.recordOutput("Drive/WheelSpeeds/Measured", getMeasuredWheelSpeeds());
		Logger.recordOutput("Drive/RobotSpeeds/Target", getTargetRobotSpeeds());
		Logger.recordOutput("Drive/RobotSpeeds/Measured", getMeasuredRobotSpeeds());

		// Get angle
		if(gyroInputs.connected) {
			yaw = gyroInputs.yaw;
		} else {
			// Calculate module deltas to guesstimate turn
			// This code is slightly modified from code in Drive.java,
			// redshift-robotics/reefscape2025
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
			for(int i = 0; i < modules.length; i++) {
				// Set current position
				modulePositions[i] = modules[i].getPosition();

				// Calculate delta
				moduleDeltas[i] = new SwerveModulePosition(
						modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
						modulePositions[i].angle);

				// Copy last position
				lastModulePositions[i] = modulePositions[i];
			}

			// Convert deltas to twist
			Twist2d twist = kinematics.toTwist2d(moduleDeltas);

			// Update yaw
			yaw = yaw.plus(new Rotation2d(twist.dtheta));
		}

		// Update the pose estimator with gyro angle and module positions
		pose = poseEstimator.update(yaw,
				Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new));
	}

}
