// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIXME: This entire file needs to be reworked.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagCoords;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Gyroscope;

/**
* Command used in teleoperation mode to AutoAim to the middle april tag
*/
public final class AutoAimFwd extends CommandBase {
	public AutoAimFwd(LimeLight inSysLimeLight, Gyroscope inSysGyroscope) {
		addRequirements(inSysLimeLight, inSysGyroscope);
	}

	boolean isAimed;

	@Override
	public void initialize() {
		isAimed = false;
	}

	@Override
	public void execute() {
		// Getting april tag coordinates from constants
		double[][] aprilTagCords = AprilTagCoords.MAP;

		// TODO: Test this.
		/**
		* The path generated by DriveTrain.genPath()
		*/
		PathPlannerTrajectory path = DriveTrain.genPath(
				1,
				.97,
				LimeLight.GetBotPose2d(),
				// Zero Yaw in Auton
				Gyroscope.sensor.getYaw(), // Should be 0 when aligned in test
				Gyroscope.sensor.getYaw(), // Theoretically should be 0 when aligned in test
				AprilTagCoords.TranslationFromAprilTagCoordinate(LimeLight.id, 1),
				aprilTagCords[(int) LimeLight.id][4], // 4 is heading in the raw cords
				aprilTagCords[(int) LimeLight.id][4]
		);

		DriveTrain.mecanumDriveOdometry.resetPosition(
				Gyroscope.sensor.getRotation2d(),
				DriveTrain.getWheelPositions(),
				new Pose2d(
						LimeLight.botPoseArray[1],
						LimeLight.botPoseArray[2],
						Gyroscope.sensor.getRotation2d()
				)
		);

		// Generates a path following command
// 		DriveTrain.followTrajectoryCommand(path, false);

		CommandScheduler.getInstance().schedule(DriveTrain.followTrajectoryCommand(path, false));

		isAimed = true;
	}

	@Override
	public boolean isFinished() {
		return isAimed;
	}
}