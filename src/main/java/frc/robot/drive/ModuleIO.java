package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIO {
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
}
