package frc.robot.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    // Distance from center of robot
    private Translation2d distanceFromCenter;

    // IO layer + inputs
    private ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    // Target state
    private SwerveModuleState targetState;

    public SwerveModule(ModuleIO io, Translation2d distanceFromCenter) {
        this.distanceFromCenter = distanceFromCenter;
        this.io = io;
    }

    // Update data going into the IO layer
    public void updateInputs() {
        // TODO
    }

    public Translation2d getDistanceFromCenter() {
        return distanceFromCenter;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.driveMotorPositionRad * Constants.WHEEL_RADIUS,
                inputs.turnMotorAbsPosition);
    }

    public void setTargetState(SwerveModuleState state) {
        // Optimize state to avoid wild spinning
        state.optimize(inputs.turnMotorAbsPosition);
        state.cosineScale(inputs.turnMotorAbsPosition);

        // TODO: IO layer comms

        targetState = state;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModuleState getMeasuredState() {
        return new SwerveModuleState(inputs.driveMotorVelocityRadPerSec * Constants.WHEEL_RADIUS,
                inputs.turnMotorAbsPosition);
    }

}
