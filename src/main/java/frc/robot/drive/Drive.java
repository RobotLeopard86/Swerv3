package frc.robot.drive;

import java.util.Arrays;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
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

	public Drive(ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br, GyroIO gyro) {
		// Set modules
		modules = new SwerveModule[] {
				new SwerveModule(fl, Constants.MODULE_FL_DISTANCE_FROM_CENTER),
				new SwerveModule(fr, Constants.MODULE_FR_DISTANCE_FROM_CENTER),
				new SwerveModule(bl, Constants.MODULE_BL_DISTANCE_FROM_CENTER),
				new SwerveModule(br, Constants.MODULE_BR_DISTANCE_FROM_CENTER),
		};

		// Configure kinematics and odometry
		kinematics = new SwerveDriveKinematics(
				Arrays.stream(modules).map(SwerveModule::getDistanceFromCenter).toArray(Translation2d[]::new));
		poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero,
				Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new),
				new Pose2d());

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
		poseEstimator.resetPosition(gyroInputs.yaw,
				Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), pose);
	}

	public void stop() {
		setTargetRobotSpeeds(new ChassisSpeeds());
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

		// Update the pose estimator with gyro angle and module positions
		pose = poseEstimator.update(gyroInputs.yaw,
				Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new));
	}

}
