package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.drive.Drive;

public class Constants {
    public enum RobotType {
        SIM
    }

    public static RobotType robot = RobotType.SIM;

    // Drive configuration
    // frameDiagonal is the diagonal length between the corners of the metal frame
    // bumperDiagnol is the same but between the corners of the bumpers
    public record DriveConfig(
            Translation2d frameDiagonal, Translation2d bumperDiagonal,
            double maxLinearVelocity, double maxLinearAcceleration) {

        public double getBaseRadius() {
            return frameDiagonal.getNorm() / 2;
        }
    }

    public static DriveConfig DRIVE_CFG = switch(robot) {
    // These values were copied from DriveConstants.java in
    // redshiftrobotics/reefscape-2025
    case SIM -> new DriveConfig(new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 0.06, 14.5);
    };

    // Module positions
    public static double CENTER_X = DRIVE_CFG.frameDiagonal.getX() / 2, CENTER_Y = DRIVE_CFG.frameDiagonal.getY() / 2;
    public static Translation2d MODULE_FL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, CENTER_Y);
    public static Translation2d MODULE_FR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, CENTER_Y);
    public static Translation2d MODULE_BL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, -CENTER_Y);
    public static Translation2d MODULE_BR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, -CENTER_Y);

    // Swerve module configuration
    public record SwerveModuleConfig(
            int driveMotorID, int turnMotorID, int absEncoder, Rotation2d absEncoderOffset, boolean turnMotorInverted) {
    }

    public static SwerveModuleConfig MODULE_CFG = switch(robot) {
    case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
    };

    // Controller gains
    public record FeedforwardGains(double kS, double kV, double kA) {
    }

    public record PIDGains(double kP, double kI, double kD) {
    }

    // Gain settings
    public static FeedforwardGains driveFfwd;
}
