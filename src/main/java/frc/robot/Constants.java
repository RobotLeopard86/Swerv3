package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

    public static final DriveConfig DRIVE_CFG = switch(robot) {
    // These values were copied from DriveConstants.java in
    // redshiftrobotics/reefscape-2025
    case SIM -> new DriveConfig(new Translation2d(0.885, 0.885), new Translation2d(0.9612, 0.9612), 0.06, 14.5);
    };

    // Module positions
    public static final double CENTER_X = DRIVE_CFG.frameDiagonal.getX() / 2;
    public static final double CENTER_Y = DRIVE_CFG.frameDiagonal.getY() / 2;
    public static final Translation2d MODULE_FL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, CENTER_Y);
    public static final Translation2d MODULE_FR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, CENTER_Y);
    public static final Translation2d MODULE_BL_DISTANCE_FROM_CENTER = new Translation2d(-CENTER_X, -CENTER_Y);
    public static final Translation2d MODULE_BR_DISTANCE_FROM_CENTER = new Translation2d(CENTER_X, -CENTER_Y);

    // Wheel radius
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

    // Swerve module configuration
    public record SwerveModuleConfig(
            int driveMotorID, int turnMotorID, int absEncoder, Rotation2d absEncoderOffset, boolean turnMotorInverted) {
    }

    public static final SwerveModuleConfig MODULE_CFG = switch(robot) {
    case SIM -> new SwerveModuleConfig(0, 0, 0, Rotation2d.kZero, false);
    };

    // Controller gains
    public record FeedforwardGains(double kS, double kV, double kA) {
    }

    public record PIDGains(double kP, double kI, double kD) {
    }

    // Gain settings
    public static final FeedforwardGains driveFeedFwd = new FeedforwardGains(0, 0, 0);
    public static final PIDGains driveFeedback = new PIDGains(0, 0, 0);
    public static final PIDGains turnFeedback = new PIDGains(0, 0, 0);

    // Reductions were copied from ModuleConstants.java in
    // redshiftrobotics/reefscape-2025!

    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    private enum Mk4iReductions {
        // Note: Mk4i turn motors are inverted!
        L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
        L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        TURN((150.0 / 7.0));

        final double reduction;

        Mk4iReductions(double reduction) {
            this.reduction = reduction;
        }
    }

    // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    private enum Mk4Reductions {
        L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
        L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        TURN((12.8 / 1.0));

        final double reduction;

        Mk4Reductions(double reduction) {
            this.reduction = reduction;
        }
    }
}
