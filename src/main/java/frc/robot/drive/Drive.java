package frc.robot.drive;

import java.util.Arrays;
import java.util.stream.Stream;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
    // Swerve modules
    SwerveModule[] modules; // FL, FR, BL, BR order

    // Kinematics & odometry
    SwerveDriveKinematics kinematics;
    SwerveDrivePoseEstimator poseEstimator;

    // Current robot pose and speeds
    Pose2d pose = new Pose2d();
    ChassisSpeeds speeds = new ChassisSpeeds();
    ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    Drive(ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        // Set modules
        modules = new SwerveModule[] {
                new SwerveModule(fl, Constants.MODULE_FL_DISTANCE_FROM_CENTER),
                new SwerveModule(fr, Constants.MODULE_FR_DISTANCE_FROM_CENTER),
                new SwerveModule(bl, Constants.MODULE_BL_DISTANCE_FROM_CENTER),
                new SwerveModule(br, Constants.MODULE_BR_DISTANCE_FROM_CENTER),
        };

        // Configure kinematics and odometry
        Stream<SwerveModule> modStream = Arrays.stream(modules);
        kinematics = new SwerveDriveKinematics(
                modStream.map(SwerveModule::getDistanceFromCenter).toArray(Translation2d[]::new));
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero,
                modStream.map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), new Pose2d());

    }

    void setTargetRobotSpeeds(ChassisSpeeds speeds) {
        // Make the speeds discrete to account for them only changing every loop period
        targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
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

    Pose2d getPose() {
        return pose;
    }

    void setPose(Pose2d pose) {
        // Reset the pose estimator
        poseEstimator.resetPosition(Rotation2d.kZero,
                Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), pose);
    }

    @Override
    public void periodic() {
        // Update the pose estimator with gyro angle (not yet implemented) and module
        // positions
        pose = poseEstimator.update(Rotation2d.kZero,
                Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new));
    }

}
