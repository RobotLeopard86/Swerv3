package frc.robot.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    private Translation2d distanceFromCenter;

    private ModuleIO io;

    public SwerveModule(ModuleIO io, Translation2d distanceFromCenter) {
        this.distanceFromCenter = distanceFromCenter;
        this.io = io;
    }

    public Translation2d getDistanceFromCenter() {
        return distanceFromCenter;
    }

    public SwerveModulePosition getPosition() {
        // TODO
        return new SwerveModulePosition();
    }
}
