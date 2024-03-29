// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * Used for driving the robot during TeleOperation by polling the controllers
 * and commanding the Mecanum with the polled controller data.
 */
public final class DriveMecanum extends Command {
	/**
	 * The robot's movement speed along the X axis, usually in a strafing motion.
	 */
	double moveSpeedX;
	/**
	 * The robot's movement speed along the Y axis, usually in forward or backward
	 * motion.
	 */
	double moveSpeedY;
	/**
	 * The robot's movement speed along the Z axis.
	 */
	double moveSpeedZ;
	/**
	 * ( :
	 */
	double speedMultiplier = 1;

	public DriveMecanum(DriveTrain inSysDriveTrain) {
		addRequirements(inSysDriveTrain);
	}

	@Override
	public void execute() {
		// TODO: Driver prefrence specific, change accordingly.
		// Get the speeds from the driver controller and multiply it by the speed.
		moveSpeedY = speedMultiplier * RobotContainer.driverOneController.getLeftX();
		moveSpeedX = -speedMultiplier * RobotContainer.driverOneController.getLeftY();
		moveSpeedZ = -speedMultiplier * RobotContainer.driverOneController.getRightX();

		// Put in controller inputs and drive the motors accordingly
		DriveTrain.mecanum.driveCartesian(moveSpeedX, moveSpeedY, moveSpeedZ);
	}

	@Override
	public void end(boolean interrupted) {
		// Set the motor speeds to zero in event of an interruption.
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
	}

}
