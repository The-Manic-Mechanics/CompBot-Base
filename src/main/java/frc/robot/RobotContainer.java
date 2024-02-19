// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controllers;
import frc.robot.commands.DriveMecanum;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
	private final Gyroscope sysGyroscope = new Gyroscope();
	private final DriveTrain sysDriveTrain = new DriveTrain();

	private final DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);

	public static final XboxController driverOneController = new XboxController(Controllers.DRIVERONE_PORT);
	// TODO: Change accordingly.
	public static final JoystickButton mainButton1 = new JoystickButton(driverOneController, 0);
	// TODO: Change accordingly.
	public static final JoystickButton mainButton2 = new JoystickButton(driverOneController, 0);

	public static final XboxController driverTwoController = new XboxController(Controllers.DRIVERTWO_PORT);
	// TODO: Change accordingly.
	public static final JoystickButton driverSecondA = new JoystickButton(driverTwoController, 0);
	// TODO: Change accordingly.
	public static final JoystickButton driverSecondY = new JoystickButton(driverTwoController, 0);
	// TODO: Change accordingly.
	public static final JoystickButton driverSecondLeftBump = new JoystickButton(driverTwoController, 0);
	// TODO: Change accordingly.
	public static final JoystickButton driverSecondRghtBump = new JoystickButton(driverTwoController, 0);

	MecanumControllerCommand mecanumController;
	public static SendableChooser<Trajectory> autonPathChooser;
	public static Pose2d initPose;

	public RobotContainer() {
		// TODO: Change accordingly.
		sysDriveTrain.setDefaultCommand(cmdDriveMecanum);
		configureBindings();

		// TODO: Change accordingly.
		initPose = new Pose2d(0, 0, new Rotation2d(0, 0));
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		// TODO: Change accordingly.
		return new InstantCommand(() -> {
		});
	}
}
