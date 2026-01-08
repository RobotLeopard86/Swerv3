package frc.robot.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.PIDGains;
import frc.robot.Constants.SwerveModuleConfig;

public class TalonFXModuleIO implements ModuleIO {
    // Module constants
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    // Motors
    private final TalonFX driveMotor, turnMotor;

    // CANcoder (encoder)
    private final CANcoder cancoder;

    // Configurations
    private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration(),
            turnMotorConfig = new TalonFXConfiguration();

    // Brake mode storage
    private boolean driveBrake = true, turnBrake = true;

    public TalonFXModuleIO(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        // Create motor and encoder objects
        driveMotor = new TalonFX(constants.DriveMotorId);
        turnMotor = new TalonFX(constants.SteerMotorId);
        cancoder = new CANcoder(constants.EncoderId);

        // Setup initial drive configuration
        driveMotorConfig.MotorOutput.NeutralMode = driveBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast; // Brake
                                                                                                                 // mode
        driveMotorConfig.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive; // Motor inversion
        driveMotorConfig.Slot0 = constants.DriveMotorGains; // Initial feedforward and feedback gains
        driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent; // Maximum allowed forward
                                                                                         // output in torque-based
                                                                                         // control mode
        driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent; // Maximum allowed reverse
                                                                                          // output in torque-based
                                                                                          // control mode
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;// Total amount of allowed current
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;// Enable the control above

        // Setup initial turn configuration
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
        turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio; // Motion Magic
                                                                                                       // (R) control
                                                                                                       // mode maximum
                                                                                                       // velocity
        turnMotorConfig.MotionMagic.MotionMagicAcceleration = turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity
                / 0.1; // Motion Magic (R) control mode target acceleration
        turnMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;// Velocity hold voltage
                                                                                              // for Motion Magic (R)
                                                                                              // Expo control modes
                                                                                              // (V/rps)
        turnMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;// Acceleration voltage for Motion Magic (R) Expo control
                                                             // modes (V/rps^2)

        // Setup initial CANcoder configuration
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        // Apply initial configurations
        driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotor.getConfigurator().apply(turnMotorConfig);
        cancoder.getConfigurator().apply(cancoderConfig);

    }
}
