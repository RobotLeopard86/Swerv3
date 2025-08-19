package frc.robot.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

	// Feedforward model
	private SimpleMotorFeedforward feedFwd;

	private String name;

	public SwerveModule(ModuleIO io, Translation2d distanceFromCenter, String name) {
		this.name = name;
		this.distanceFromCenter = distanceFromCenter;
		this.io = io;

		// Initialize feedforward model
		feedFwd = new SimpleMotorFeedforward(Constants.DRIVE_FEEDFWD_GAINS.kS(), Constants.DRIVE_FEEDFWD_GAINS.kV(),
				Constants.DRIVE_FEEDFWD_GAINS.kA(), Constants.LOOP_PERIOD);

		// Configure PID gains
		io.setDriveMotorPIDGains(Constants.DRIVE_PID_GAINS);
		io.setTurnMotorPIDGains(Constants.TURN_PID_GAINS);

		// Enable brake by default
		io.setDriveMotorBrake(true);
		io.setTurnMotorBrake(true);
	}

	// Update data going into the IO layer
	public void updateInputs() {
		Logger.processInputs("Drive/Module" + name, inputs);
		io.updateInputs(inputs);
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

		// Set motor values
		io.setDriveMotorVelocity(state.speedMetersPerSecond / Constants.WHEEL_RADIUS,
				feedFwd.calculate(state.speedMetersPerSecond));
		io.setTurnMotorPosition(state.angle.getRadians());

		targetState = state;
	}

	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState(inputs.driveMotorVelocityRadPerSec * Constants.WHEEL_RADIUS,
				inputs.turnMotorAbsPosition);
	}

	public void stop() {
		io.stop();
	}

}
