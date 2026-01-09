package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.FFwdGains;
import frc.robot.Constants.PIDGains;

public class SimModuleIO implements ModuleIO {

	// Motors
	private final DCMotor driveMotor = DCMotor.getKrakenX60Foc(1), turnMotor = DCMotor.getKrakenX60Foc(1);

	// Motor simulators
	private final DCMotorSim driveSim, turnSim;

	// Voltages
	private double driveVolts = 0.0, driveFFwdVolts = 0.0, turnVolts = 0.0;

	// Feedback controllers
	private final PIDController drivePID, turnPID;

	// Feedforward models
	private final SimpleMotorFeedforward driveFFwd;

	// Closed-loop control?
	private boolean driveUseClosedLoop = false, turnUseClosedLoop = false;

	public SimModuleIO() {
		// Note: JKgMetersSquared values copied from ModuleIOSim.java in
		// redshiftrobotics/reefscape-2025
		// I looked and couldn't find good resources on how to calculate it myself
		driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveMotor, 0.025, Constants.DRIVE_REDUCTION),
				driveMotor);
		turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(turnMotor, 0.004, Constants.TURN_REDUCTION),
				turnMotor);

		// Create PID controllers
		drivePID = new PIDController(0.0, 0.0, 0.0);
		turnPID = new PIDController(0.0, 0.0, 0.0);

		// Create feedforward model
		driveFFwd = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

		// Set turn PID to allow continuous motion
		turnPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		// Drive motor feedback
		if(driveUseClosedLoop) {
			// Update the voltage with the PID controller output
			driveVolts = driveFFwdVolts + drivePID.calculate(driveSim.getAngularVelocityRadPerSec());
		} else {
			// Reset the PID controller to make sure it doesn't get outdated
			drivePID.reset();
		}

		// Turn motor feedback
		if(turnUseClosedLoop) {
			// Update the voltage with the PID controller output
			turnVolts = turnPID.calculate(turnSim.getAngularPositionRad());
		} else {
			// Reset the PID controller to make sure it doesn't get outdated
			turnPID.reset();
		}

		// Update simulated motor voltages
		driveSim.setInputVoltage(MathUtil.clamp(driveVolts, -12.0f, 12.0f));
		turnSim.setInputVoltage(MathUtil.clamp(turnVolts, -12.0f, 12.0f));

		// Update simulated motor state
		driveSim.update(Constants.LOOP_PERIOD);
		turnSim.update(Constants.LOOP_PERIOD);

		// Update input state
		inputs.driveMotorAppliedVolts = driveVolts;
		inputs.driveMotorCurrentAmpsSupply = Math.abs(driveSim.getCurrentDrawAmps());
		inputs.driveMotorPositionRad = driveSim.getAngularPositionRad();
		inputs.driveMotorVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveMotorConnected = true;
		inputs.turnMotorAppliedVolts = turnVolts;
		inputs.turnMotorCurrentAmpsSupply = Math.abs(turnSim.getCurrentDrawAmps());
		inputs.turnMotorPosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
		inputs.turnMotorAbsPosition = inputs.turnMotorPosition;
		inputs.turnMotorVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnMotorConnected = true;
		inputs.turnAbsEncoderConnected = true;
	}

	@Override
	public void setDriveMotorBrake(boolean brake) {
	}

	@Override
	public void setDriveMotorPIDGains(PIDGains gains) {
		drivePID.setPID(gains.kP(), gains.kI(), gains.kD());
	}

	@Override
	public void setDriveMotorFFwdGains(FFwdGains gains) {
		driveFFwd.setKa(gains.kA());
		driveFFwd.setKs(gains.kS());
		driveFFwd.setKv(gains.kV());
	}

	@Override
	public void setDriveMotorVelocity(double velocityRadPerSec) {
		// Now using feedback control to set the velocity, not manual voltage control,
		// so we want closed-loop mode
		driveUseClosedLoop = true;

		// Set velocity setpoint
		drivePID.setSetpoint(velocityRadPerSec);

		// Set feedforward voltage
		driveFFwdVolts = driveFFwd.calculate(velocityRadPerSec);
	}

	@Override
	public void setDriveMotorVoltage(double voltage) {
		// Now we're setting voltage directly, so we don't want feedback
		// Thus, we disable the closed-loop mode
		driveUseClosedLoop = false;

		// Set the voltage
		driveVolts = voltage;
	}

	@Override
	public void setTurnMotorBrake(boolean brake) {
	}

	@Override
	public void setTurnMotorPIDGains(PIDGains gains) {
		turnPID.setPID(gains.kP(), gains.kI(), gains.kD());
	}

	@Override
	public void setTurnMotorPosition(double positionRad) {
		// Now using feedback control to set the position, not manual voltage control,
		// so we want closed-loop mode
		turnUseClosedLoop = true;

		// Set position setpoint
		turnPID.setSetpoint(positionRad);
	}

	@Override
	public void setTurnMotorVoltage(double voltage) {
		// Now we're setting voltage directly, so we don't want feedback
		// Thus, we disable the closed-loop mode
		turnUseClosedLoop = false;

		// Set the voltage
		turnVolts = voltage;
	}

	@Override
	public void stop() {
		// Stop the motors
		setDriveMotorVoltage(0);
		setTurnMotorVoltage(0);
	}

}
