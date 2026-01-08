// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.Drive;
import frc.robot.drive.DriveCommands;
import frc.robot.drive.GyroIO;
import frc.robot.drive.SimModuleIO;
import frc.robot.drive.TalonFXModuleIO;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
	// Controller
	private CommandXboxController xbox = new CommandXboxController(Constants.XBOX_PORT);
	private JoystickInputHelper joystickHelper;

	// Drive subsystem
	private Drive drive;

	public RobotContainer() {
		// Select appropriate IO layers for drive
		drive = switch(Constants.ROBOT_TYPE) {
		case SIM -> new Drive(new SimModuleIO(), new SimModuleIO(), new SimModuleIO(), new SimModuleIO(), new GyroIO() {
		});
		case PRESEASON_2026 -> new Drive(new TalonFXModuleIO(TunerConstants.FrontLeft),
				new TalonFXModuleIO(TunerConstants.FrontRight), new TalonFXModuleIO(TunerConstants.BackLeft),
				new TalonFXModuleIO(TunerConstants.BackRight), new GyroIO() {
				});
		};

		configureBindings();
	}

	private void configureBindings() {
		// Set up joystick helper
		joystickHelper = new JoystickInputHelper(() -> -xbox.getLeftY(), () -> -xbox.getLeftX(),
				() -> -xbox.getRightX());

		// Configure drive command
		drive.setDefaultCommand(
				DriveCommands.teleopDrive(drive, () -> joystickHelper.getTranslation(),
						() -> joystickHelper.getOmega()));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
