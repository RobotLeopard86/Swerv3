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

public class RobotContainer {
	// Controller
	private CommandXboxController xbox = new CommandXboxController(Constants.XBOX_PORT);
	private JoystickInputHelper joyHlpr;

	// Drive subsystem
	private Drive drive;

	public RobotContainer() {
		// Select appropriate IO layers for drive
		drive = switch(Constants.ROBOT_TYPE) {
		case SIM -> new Drive(new SimModuleIO(), new SimModuleIO(), new SimModuleIO(), new SimModuleIO(), new GyroIO() {
		});
		};

		configureBindings();
	}

	private void configureBindings() {
		// Set up joystick helper
		joyHlpr = new JoystickInputHelper(() -> -xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightY());

		// Configure drive command
		drive.setDefaultCommand(
				DriveCommands.teleopDrive(drive, () -> joyHlpr.getTranslation(), () -> joyHlpr.getOmega()));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
