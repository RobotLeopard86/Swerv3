package frc.robot.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.FFwdGains;
import frc.robot.Constants.PIDGains;
import frc.robot.generated.TunerConstants;

public class TalonFXModuleIO implements ModuleIO {
	// Module constants
	private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

	// Motors
	private final TalonFX driveMotor, turnMotor;

	// CANcoder (encoder)
	private final CANcoder cancoder;

	// Configurations
	private final TalonFXConfiguration driveMotorConfig, turnMotorConfig;
	private final CANcoderConfiguration cancoderConfig;

	// Brake mode storage
	private boolean driveBrake = true, turnBrake = true;

	// Data signals
	private final StatusSignal<Angle> drivePosition, turnPosition, turnAbsPosition;
	private final StatusSignal<AngularVelocity> driveVelocity, turnVelocity;
	private final StatusSignal<Voltage> driveVoltage, turnVoltage;
	private final StatusSignal<Current> driveCurrent, turnCurrent;

	// Connection status debouncers
	private final Debouncer driveConnectionDebouncer = new Debouncer(
			Constants.DRIVE_MOTOR_DISCONNECT_WARNING_THRESHOLD_SECONDS);
	private final Debouncer turnConnectionDebouncer = new Debouncer(
			Constants.DRIVE_MOTOR_DISCONNECT_WARNING_THRESHOLD_SECONDS);
	private final Debouncer cancoderConnectionDebouncer = new Debouncer(
			Constants.DRIVE_ENCODER_DISCONNECT_WARNING_THRESHOLD_SECONDS);

	// Voltage control requests
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

	// Torque current control requests
	private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
	private final PositionTorqueCurrentFOC positionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0.0);
	private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0.0);

	public TalonFXModuleIO(
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
		this.constants = constants;

		// Create motor and encoder objects
		driveMotor = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
		turnMotor = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
		cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

		// Setup initial drive configurationr
		driveMotorConfig = constants.DriveMotorInitialConfigs.clone();
		driveMotorConfig.MotorOutput.NeutralMode = driveBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast; // Brake
		// mode
		driveMotorConfig.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive; // Motor inversion
		driveMotorConfig.Slot0 = constants.DriveMotorGains; // Initial feedforward and feedback gains
		driveMotorConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio; // Gear ratio of motor
		driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent; // Maximum allowed
		// forward
		// output in
		// torque-based
		// control mode
		driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent; // Maximum allowed
		// reverse
		// output in
		// torque-based
		// control mode
		driveMotorConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;// Total amount of allowed
		// current
		driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;// Enable the control above

		// Setup initial turn configuration
		turnMotorConfig = constants.SteerMotorInitialConfigs.clone();
		turnMotorConfig.MotorOutput.NeutralMode = turnBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;// Brake
		// mode
		turnMotorConfig.MotorOutput.Inverted = constants.SteerMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;// Motor inversion
		turnMotorConfig.Slot0 = constants.SteerMotorGains;// Initial feedforward and feedback gains
		turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;// Enable continuous motor input
		turnMotorConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;// Feedback sensor (encoder) ID
		turnMotorConfig.Feedback.FeedbackSensorSource = switch(constants.FeedbackSource) {
			case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
			case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
			case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
			default -> throw new RuntimeException("Unsupported turn motor feedback source!");
		}; // Sensor source type
		turnMotorConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;// Gear ratio of motor
		turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio; // Motion
		// Magic
		// (R)
		// control
		// mode
		// maximum
		// velocity
		turnMotorConfig.MotionMagic.MotionMagicAcceleration = turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity
				/ 0.1; // Motion Magic (R) control mode target acceleration
		turnMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;// Velocity hold
		// voltage
		// for Motion
		// Magic (R)
		// Expo control
		// modes
		// (V/rps)
		turnMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;// Acceleration voltage for Motion Magic (R) Expo
		// control
		// modes (V/rps^2)

		// Setup initial CANcoder configuration
		cancoderConfig = constants.EncoderInitialConfigs.clone();
		cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
		cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
				? SensorDirectionValue.Clockwise_Positive
				: SensorDirectionValue.CounterClockwise_Positive;

		// Apply initial configurations
		driveMotor.getConfigurator().apply(driveMotorConfig);
		turnMotor.getConfigurator().apply(turnMotorConfig);
		cancoder.getConfigurator().apply(cancoderConfig);

		// Fetch status signals
		drivePosition = driveMotor.getPosition();
		driveVelocity = driveMotor.getVelocity();
		driveVoltage = driveMotor.getMotorVoltage();
		driveCurrent = driveMotor.getStatorCurrent();
		turnPosition = turnMotor.getPosition();
		turnAbsPosition = cancoder.getAbsolutePosition();
		turnVelocity = turnMotor.getVelocity();
		turnVoltage = turnMotor.getMotorVoltage();
		turnCurrent = turnMotor.getStatorCurrent();

		// Configure data polling behavior
		BaseStatusSignal.setUpdateFrequencyForAll(Constants.DRIVE_ODOMETRY_FREQUENCY_HZ, drivePosition,
				turnPosition);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveVoltage, driveCurrent,
				turnAbsPosition, turnVelocity, turnVoltage, turnCurrent);
		ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		// Refresh status signals with current data
		StatusCode driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveVoltage,
				driveCurrent);
		StatusCode turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnVoltage,
				turnCurrent);
		StatusCode cancoderStatus = BaseStatusSignal.refreshAll(turnAbsPosition);

		// Set connection values
		inputs.driveMotorConnected = driveConnectionDebouncer.calculate(driveStatus.isOK());
		inputs.turnMotorConnected = turnConnectionDebouncer.calculate(turnStatus.isOK());
		inputs.turnAbsEncoderConnected = cancoderConnectionDebouncer.calculate(cancoderStatus.isOK());

		// Update drive inputs
		inputs.driveMotorPositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
		inputs.driveMotorVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
		inputs.driveMotorAppliedVolts = driveVoltage.getValueAsDouble();
		inputs.driveMotorCurrentAmpsSupply = driveCurrent.getValueAsDouble();

		// Update turn inputs
		inputs.turnMotorPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnMotorAbsPosition = Rotation2d.fromRotations(turnAbsPosition.getValueAsDouble());
		inputs.turnMotorVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
		inputs.turnMotorAppliedVolts = turnVoltage.getValueAsDouble();
		inputs.turnMotorCurrentAmpsSupply = turnCurrent.getValueAsDouble();
	}

	@Override
	public void setDriveMotorBrake(boolean brake) {
		driveBrake = brake;
		driveMotorConfig.MotorOutput.NeutralMode = driveBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		driveMotor.getConfigurator().apply(driveMotorConfig);
	}

	@Override
	public void setDriveMotorPIDGains(PIDGains gains) {
		driveMotorConfig.Slot0.kP = gains.kP();
		driveMotorConfig.Slot0.kI = gains.kI();
		driveMotorConfig.Slot0.kD = gains.kD();
		driveMotor.getConfigurator().apply(driveMotorConfig);
	}

	@Override
	public void setDriveMotorFFwdGains(FFwdGains gains) {
		driveMotorConfig.Slot0.kA = gains.kA();
		driveMotorConfig.Slot0.kS = gains.kS();
		driveMotorConfig.Slot0.kV = gains.kV();
		driveMotor.getConfigurator().apply(driveMotorConfig);
	}

	@Override
	public void setDriveMotorVelocity(double velocityRadPerSec) {
		driveMotor.setControl(switch(constants.DriveMotorClosedLoopOutput) {
			case Voltage -> velocityVoltageRequest
					.withVelocity(Units.radiansToRotations(velocityRadPerSec));
			case TorqueCurrentFOC -> velocityTorqueCurrentFOCRequest
					.withVelocity(Units.radiansToRotations(velocityRadPerSec));
		});
	}

	@Override
	public void setDriveMotorVoltage(double voltage) {
		driveMotor.setControl(switch(constants.DriveMotorClosedLoopOutput) {
			case Voltage -> voltageRequest.withOutput(voltage);
			case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(voltage);
		});
	}

	@Override
	public void setTurnMotorBrake(boolean brake) {
		turnBrake = brake;
		turnMotorConfig.MotorOutput.NeutralMode = turnBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		turnMotor.getConfigurator().apply(turnMotorConfig);
	}

	@Override
	public void setTurnMotorPIDGains(PIDGains gains) {
		turnMotorConfig.Slot0.kP = gains.kP();
		turnMotorConfig.Slot0.kI = gains.kI();
		turnMotorConfig.Slot0.kD = gains.kD();
		turnMotor.getConfigurator().apply(turnMotorConfig);
	}

	@Override
	public void setTurnMotorFFwdGains(FFwdGains gains) {
		turnMotorConfig.Slot0.kA = gains.kA();
		turnMotorConfig.Slot0.kS = gains.kS();
		turnMotorConfig.Slot0.kV = gains.kV();
		turnMotor.getConfigurator().apply(turnMotorConfig);
	}

	@Override
	public void setTurnMotorPosition(double positionRad) {
		turnMotor.setControl(switch(constants.SteerMotorClosedLoopOutput) {
			case Voltage -> positionVoltageRequest
					.withVelocity(Units.radiansToRotations(positionRad));
			case TorqueCurrentFOC -> positionTorqueCurrentFOCRequest
					.withVelocity(Units.radiansToRotations(positionRad));
		});
	}

	@Override
	public void setTurnMotorVoltage(double voltage) {
		// Stop motor if 0 voltage provided
		if(voltage == 0) {
			turnMotor.stopMotor();
			return;
		}

		// Apply target voltage
		turnMotor.setControl(switch(constants.SteerMotorClosedLoopOutput) {
			case Voltage -> voltageRequest.withOutput(voltage);
			case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(voltage);
		});
	}

	@Override
	public void stop() {
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}

}
