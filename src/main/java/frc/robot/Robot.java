// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotType;

public class Robot extends LoggedRobot {
	private Command autoCmd;

	private final RobotContainer robotContainer;

	public Robot() {
		// Log metadata
		Logger.recordMetadata("Project", "Swerv3 by RobotLeopard86");
		Logger.recordMetadata("BuildInfo/Date", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("BuildInfo/Commit", BuildConstants.GIT_SHA);
		Logger.recordMetadata("BuildInfo/CommitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitState", switch(BuildConstants.DIRTY) {
			case 0 -> "All changes committed - working tree clean";
			case 1 -> "Uncommitted changes";
			default -> "Unknown";
		});

		// Configure & start AdvantageKit
		// Logic copied from Robot.java, redshiftrobotics/reefscape-2025
		switch(Constants.ENV) {
			case REALITY:
			// Running on a real robot, log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new WPILOGWriter());
			Logger.addDataReceiver(new NT4Publisher());
				break;

			case SIM:
			// Running a physics simulator, log to NT
			Logger.addDataReceiver(new NT4Publisher());
				break;

			case REPLAY:
			// Replaying a log, set up replay source
			setUseTiming(false); // Run as fast as possible
			final String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}
		Logger.start();

		// Ensure consistent alliance settings
		if(Constants.ROBOT_TYPE == RobotType.SIM) {
			DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
			DriverStationSim.notifyNewData();
		}

		// Create robot container
		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		autoCmd = robotContainer.getAutonomousCommand();

		if(autoCmd != null) {
			autoCmd.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if(autoCmd != null) {
			autoCmd.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
