// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import frc.robot.Constants.DriveTrain.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.*;

/**
* The subsystem that holds all the motors and functions pertaining to the DriveTrain
*/
public final class DriveTrain extends SubsystemBase {
	/**
	* Represents the MecanumDrive (Used for driving the robot)
	*/
	public static MecanumDrive mecanum;

	public static class Motors {
		public static WPI_VictorSPX frontLeft, frontRight, rearLeft, rearRight;
	}

	public static class Encoders {
		public static Encoder frontLeft, frontRight, rearLeft, rearRight;
	}

	public static class Odometry {
		/**
		* Used for keeping track of the robot's position while it drives
		*/
		public static MecanumDriveOdometry mecanumDriveOdometry;

		public static Consumer<Pose2d> resetDriveOdometry() {
			// if (LimeLight.tagID == 0)
				mecanumDriveOdometry.resetPosition(Gyroscope.sensor.getRotation2d(), Kinematics.getWheelPositions(), mecanumDriveOdometry.getPoseMeters());
			// else
				// mecanumDriveOdometry.resetPosition(Gyroscope.sensor.getRotation2d(), Kinematics.getWheelPositions(), LimeLight.getBotPose2d());
			return (Consumer<Pose2d>)new Pose2d(mecanumDriveOdometry.getPoseMeters().getX(), mecanumDriveOdometry.getPoseMeters().getY(), Gyroscope.sensor.getRotation2d());
		}
		
	}

	public static class Kinematics {
		/**
		* Used for tracking how far the robot actually goes compared to what values are being inputed
		*/
		public static MecanumDriveKinematics mecanumDriveKinematics;
		/**
		* Stores the position of the wheels in robot space (Used for kinematics and odometry)
		*/
		private static MecanumDriveWheelPositions wheelPositions;
		/**
		 * The current speed of the robot as a ChasisSpeeds
		 */
		private static ChassisSpeeds robotSpeed;
		/**
		 * Stores the speed of the wheels
		 */
		public static MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds;

		@SuppressWarnings ("unchecked")
		public static Supplier<ChassisSpeeds> getMecanumChassisSpeeds() {
			return (Supplier<ChassisSpeeds>)DriveTrain.Kinematics.mecanumDriveKinematics.toChassisSpeeds(DriveTrain.Kinematics.mecanumDriveWheelSpeeds);
		}

		/**
		 * Gets the current mecanum wheel positions.
		 *
		 * @return <i>(type Mecanum`riveWheelPositions)</i> filled out
		*/
		public static MecanumDriveWheelPositions getWheelPositions() {
			return new MecanumDriveWheelPositions(
					DriveTrain.Encoders.frontLeft.getDistance(), DriveTrain.Encoders.frontRight.getDistance(),
					DriveTrain.Encoders.rearLeft.getDistance(), DriveTrain.Encoders.rearRight.getDistance());
		}
	}

	public DriveTrain() {

		// ----------------------------
		// Motors
		// ----------------------------

		Motors.frontLeft = new WPI_VictorSPX(MotorPorts.FRONT_LEFT);
		Motors.frontRight = new WPI_VictorSPX(MotorPorts.FRONT_RIGHT);
		Motors.rearLeft = new WPI_VictorSPX(MotorPorts.BACK_LEFT);
		Motors.rearRight = new WPI_VictorSPX(MotorPorts.BACK_RIGHT);

		Motors.frontLeft.setInverted(true);
		Motors.rearLeft.setInverted(true);

		// ------------------------------------------------------------

		// ----------------------------
		// Encoders
		// ----------------------------

		Encoders.frontLeft = new Encoder(
				EncoderPorts.FRONT_LEFT_A,
				EncoderPorts.FRONT_LEFT_B
		);

		Encoders.frontRight = new Encoder(
				EncoderPorts.FRONT_RIGHT_A,
				EncoderPorts.FRONT_RIGHT_B
		);

		Encoders.rearLeft = new Encoder(
				EncoderPorts.BACK_LEFT_A,
				EncoderPorts.BACK_LEFT_B
		);

		Encoders.rearRight = new Encoder(
				EncoderPorts.BACK_RIGHT_A,
				EncoderPorts.BACK_RIGHT_B
		);
		Encoders.frontLeft.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.frontRight.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.rearLeft.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.rearRight.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);

		// ---------------------------------------------------------------

		// ----------------------
		// MecanumDrive object
		// ----------------------

		mecanum = new MecanumDrive(Motors.frontLeft, Motors.rearLeft, Motors.frontRight, Motors.rearRight);

		// ------------------------------------------------------------

		// --------------------------
		// Kinematics
		// --------------------------

		Kinematics.mecanumDriveKinematics = new MecanumDriveKinematics
	    (
				new Translation2d(MotorLocations.FRONT_LEFT, MotorLocations.FRONT_LEFT),
				new Translation2d(MotorLocations.FRONT_RIGHT, -1 * MotorLocations.FRONT_RIGHT),
				new Translation2d(-1 * MotorLocations.BACK_LEFT, MotorLocations.BACK_LEFT),
				new Translation2d(-1 * MotorLocations.BACK_RIGHT, -1 * MotorLocations.BACK_RIGHT)
		);

		Kinematics.wheelPositions = new MecanumDriveWheelPositions(
				Encoders.frontLeft.getDistance(),
				Encoders.frontRight.getDistance(),
				Encoders.rearLeft.getDistance(),
				Encoders.rearRight.getDistance()
		);

		Kinematics.mecanumDriveWheelSpeeds = new MecanumDriveWheelSpeeds(
				Motors.frontLeft.get(), 
				Motors.frontRight.get(), 
				Motors.rearLeft.get(), 
				Motors.rearRight.get()
		);

		// -----------------------------------------------------------

		// ----------------------------
		// Limelight Pose
		// ----------------------------

		// double[] currentPose = LimeLight.botPoseArray;

		Pose2d initPose = new Pose2d(0,0/*currentPose[1], currentPose[2]*/, Gyroscope.sensor.getRotation2d());

		// ------------------------------------------------------------------

		// -----------------------------
		// Odometry
		// -----------------------------

		Odometry.mecanumDriveOdometry = new MecanumDriveOdometry(Kinematics.mecanumDriveKinematics, Gyroscope.sensor.getRotation2d(), Kinematics.wheelPositions, initPose);
	
		// ------------------------------------------------------------------
	}

    @Override
	public void periodic() {
		// This method will be called once per scheduler run
		Odometry.mecanumDriveOdometry.update(Gyroscope.sensor.getRotation2d(), Kinematics.wheelPositions);

		SmartDashboard.putNumber("X Value", RobotContainer.driverOneController.getLeftX());
		SmartDashboard.putNumber("Y Value", RobotContainer.driverOneController.getLeftY());
		SmartDashboard.putNumber("Z Value", RobotContainer.driverOneController.getRightX());
	}
}
