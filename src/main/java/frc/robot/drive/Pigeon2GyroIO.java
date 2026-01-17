package frc.robot.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class Pigeon2GyroIO implements GyroIO {

	// Pigeon2 (the gyro)
	private final Pigeon2 pigeon;

	// Configuration
	private final Pigeon2Configuration pigeonConfig;

	// Status signals
	private final StatusSignal<Angle> yaw;
	private final StatusSignal<AngularVelocity> yawVelocity;

	// Connection debouncer
	private final Debouncer connectionDebouncer = new Debouncer(
			Constants.DRIVE_ENCODER_DISCONNECT_WARNING_THRESHOLD_SECONDS);

	public Pigeon2GyroIO() {
		// Setup Pigeon2 and config
		pigeon = new Pigeon2(Constants.GYRO_ID);
		pigeonConfig = new Pigeon2Configuration();
		pigeon.getConfigurator().apply(pigeonConfig);

		// Fetch status signals
		yaw = pigeon.getYaw();
		yawVelocity = pigeon.getAngularVelocityZWorld();

		// Set initial yaw
		pigeon.setYaw(0);

		// Configure status signal polling behavior
		yaw.setUpdateFrequency(Constants.DRIVE_ODOMETRY_FREQUENCY_HZ);
		yawVelocity.setUpdateFrequency(50.0);
		pigeon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		// Refresh status signals with current data
		StatusCode status = BaseStatusSignal.refreshAll(yaw, yawVelocity);

		// Set values
		inputs.connected = connectionDebouncer.calculate(status.isOK());
		inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
	}

	@Override
	public void reset() {
		pigeon.reset();
	}
}
