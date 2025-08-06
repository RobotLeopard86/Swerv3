// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.Drive;

public class RobotContainer {
	// Controller
	private CommandXboxController xbox = new CommandXboxController(Constants.XBOX_PORT);

	// Drive subsystem
	private Drive drive;

	public RobotContainer() {
		// Select appropriate IO
		switch(Constants.ROBOT_TYPE) {
		case SIM:
			// TODO: Need to implement SIM IO first
			break;
		}

		configureBindings();
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
