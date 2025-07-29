package frc.robot.drive;

import java.util.Arrays;
import java.util.stream.Stream;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
    // Swerve modules
    SwerveModule moduleFL, moduleFR, moduleBL, moduleBR;

    // Kinematics & odometry
    SwerveDriveKinematics kinematics;
    SwerveDrivePoseEstimator poseEstimator;

    Drive(ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        // Set modules
        moduleFL = new SwerveModule(fl, Constants.MODULE_FL_DISTANCE_FROM_CENTER);
        moduleFR = new SwerveModule(fr, Constants.MODULE_FR_DISTANCE_FROM_CENTER);
        moduleBL = new SwerveModule(bl, Constants.MODULE_BL_DISTANCE_FROM_CENTER);
        moduleBR = new SwerveModule(br, Constants.MODULE_BR_DISTANCE_FROM_CENTER);

        // Configure kinematics and odometry
        Stream<SwerveModule> modules = Arrays.stream(new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR });
        kinematics = new SwerveDriveKinematics(
                modules.map(SwerveModule::getDistanceFromCenter).toArray(Translation2d[]::new));
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero,
                modules.map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), new Pose2d());
    }
}
