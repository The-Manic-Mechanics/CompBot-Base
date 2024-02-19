// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public final class Constants {
	public static class Controllers {
		/**
		 * TODO: Change accordingly.
		 * The port which the first driver's controller is connected to.
		 */
		public static final int DRIVERONE_PORT = 0;
		/**
		 * TODO: Change accordingly.
		 * The port which the second driver's controller is connected to.
		 */
		public static final int DRIVERTWO_PORT = 0;
	}

	public static class LimeLightMounting {
		/**
		 * TODO: Change accordingly.
		 * How many degrees the limelight is mounted from perfectly vertical.
		 */
		public static final double limelightMountAngleDegrees = 0;
		/**
		 * TODO: Change accordingly.
		 * The amount of inches from the center of the LimeLight lens to the floor.
		 */
		public static final double lensHeightInches = 0;
	}

	public static class Gyroscope {
		public static final Port gyroPort = SPI.Port.kMXP;
	}

	public static class DriveTrain {
		public static class MotorPorts {
			/**
			 * TODO: Change accordingly.
			 * The CAN port of the front left motor.
			 */
			public static final int FRONT_LEFT = 0;
			/**
			 * TODO: Change accordingly.
			 * The CAN port of the front right motor.
			 */
			public static final int FRONT_RIGHT = 0;
			/**
			 * TODO: Change accordingly.
			 * The CAN port of the back left motor.
			 */
			public static final int BACK_LEFT = 0;
			/**
			 * TODO: Change accordingly.
			 * The CAN port of the back right motor.
			 */
			public static final int BACK_RIGHT = 0;
		}

		public static class EncoderPorts {
			/**
			 * TODO: Change accordingly.
			 * The A channel port of the front left encoder.
			 */
			public static final int FRONT_LEFT_A = 0;
			/**
			 * TODO: Change accordingly.
			 * The B channel port of the front left encoder.
			 */
			public static final int FRONT_LEFT_B = 0;
			/**
			 * TODO: Change accordingly.
			 * The A channel port of the front right encoder.
			 */
			public static final int FRONT_RIGHT_A = 0;
			/**
			 * TODO: Change accordingly.
			 * The B channel port of the front right encoder.
			 */
			public static final int FRONT_RIGHT_B = 0;
			/**
			 * TODO: Change accordingly.
			 * The A channel port of the back left encoder.
			 */
			public static final int BACK_LEFT_A = 0;
			/**
			 * TODO: Change accordingly.
			 * The B channel port of the back left encoder.
			 */
			public static final int BACK_LEFT_B = 0;
			/**
			 * TODO: Change accordingly.
			 * The A channel port of the back right encoder.
			 */
			public static final int BACK_RIGHT_A = 0;
			/**
			 * TODO: Change accordingly.
			 * The B channel port of the back right encoder.
			 */
			public static final int BACK_RIGHT_B = 0;
		}

		public static class MotorLocations {
			/**
			 * TODO: Change accordingly.
			 * The position of the front left wheel relative to the center of the robot, in
			 * meters.
			 */
			public static final double FRONT_LEFT = 0;
			/**
			 * TODO: Change accordingly.
			 * The position of the front right wheel relative to the center of the robot, in
			 * meters.
			 */
			public static final double FRONT_RIGHT = 0;
			/**
			 * TODO: Change accordingly.
			 * The position of the back left wheel relative to the center of the robot, in
			 * meters.
			 */
			public static final double BACK_LEFT = 0;
			/**
			 * TODO: Change accordingly.
			 * The position of the back right wheel relative to the center of the robot, in
			 * meters.
			 */
			public static final double BACK_RIGHT = 0;
		}
	}

	public static class Auton {
		/**
		 * TODO: Change accordingly.
		 * The max forward speed of the robot in meters per second.
		 */
		public static final double MAX_SPEED = 0;
		/**
		 * TODO: Change accordingly.
		 * The prefered velocity of the robot in autonomous mode in meters per second.
		 */
		public static final double DRIVE_VEL = 0;
		/**
		 * TODO: Change accordingly.
		 * The distance between the left and right wheels in meters.
		 */
		public static final double TRACK_WIDTH_METERS = 0;
		/**
		 * TODO: Change accordingly.
		 * The distance between pulses of the encoders in feet.
		 */
		public static final double DISTANCE_PER_PULSE = 0;
		/**
		 * TODO: Change accordingly.
		 * The maximum acceleration of the robot in meters per second.
		 */
		public static final double MAX_ACCEL = 0;

		public static class FeedForwardControllers {
			/**
			 * TODO: Change accordingly.
			 * The static gain, determined by a SysID characterization.
			 */
			public static final double STATIC_GAIN = 0;
			/**
			 * TODO: Change accordingly.
			 * The velocity gain, determined by a SysID characterization.
			 */
			public static final double VELOCITY_GAIN = 0;
			/**
			 * TODO: Change accordingly.
			 * The acceleration gain, determined by a SysID characterization.
			 */
			public static final double ACCEL_GAIN = 0;
		}

		public static class PIDControllers {
			public static class Holonomic {
				/**
				 * TODO: Change accordingly.
				 * The P constant for the holonomicController's X correction.
				 * PID loop.
				 * Position units are field relative.
				 */
				public static final double XCONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the holonomicController's X correction.
				 * PID loop.
				 */
				public static final double XCONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the holonomicController's X correction.
				 * PID loop.
				 * Position units are field relative.
				 */
				public static final double XCONTROLLER_D = 0;
				/**
				 * TODO: Change accordingly.
				 * The P constant for the holonomicController's Y correction.
				 * PID loop.
				 * Position units are field relative.
				 */
				public static final double YCONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the holonomicController's Y correction.
				 * PID loop.
				 * Position units are field relative.
				 */
				public static final double YCONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the holonomicController's Y correction.
				 * PID loop.
				 * Position units are field relative.
				 */
				public static final double YCONTROLLER_D = 0;
				/**
				 * TODO: Change accordingly.
				 * The P constant for the holonomicController's rotation correction PID loop.
				 * Units are in degrees.
				 */
				public static final double THETACONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the holonomicController's rotation correction PID loop.
				 * Units are in degrees.
				 */
				public static final double THETACONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the holonomicController's rotation correction PID loop.
				 * Units are in degrees.
				 */
				public static final double THETACONTROLLER_D = 0;
			}

			public static class WheelVelocities {
				/**
				 * TODO: Change accordingly.
				 * The P constant for the individual motor velocity PIDController of the
				 * frontLeft motor.
				 */
				public static final double FL_CONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the individual motor velocity PIDController of the
				 * frontLeft motor.
				 */
				public static final double FL_CONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the individual motor velocity PIDController of the
				 * frontLeft motor.
				 */
				public static final double FL_CONTROLLER_D = 0;
				/**
				 * TODO: Change accordingly.
				 * The P constant for the individual motor velocity PIDController of the
				 * frontRight motor.
				 */
				public static final double FR_CONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the individual motor velocity PIDController of the
				 * frontRight motor.
				 */
				public static final double FR_CONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the individual motor velocity PIDController of the
				 * frontRight motor.
				 */
				public static final double FR_CONTROLLER_D = 0;
				/**
				 * TODO: Change accordingly.
				 * The P constant for the individual motor velocity PIDController of the
				 * rearLeft motor.
				 */
				public static final double RL_CONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the individual motor velocity PIDController of the
				 * rearLeft motor.
				 */
				public static final double RL_CONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the individual motor velocity PIDController of the
				 * rearLeft motor.
				 */
				public static final double RL_CONTROLLER_D = 0;
				/**
				 * TODO: Change accordingly.
				 * The P constant for the individual motor velocity PIDController of the
				 * rearRight motor.
				 */
				public static final double RR_CONTROLLER_P = 0;
				/**
				 * TODO: Change accordingly.
				 * The I constant for the individual motor velocity PIDController of the
				 * rearRight motor.
				 */
				public static final double RR_CONTROLLER_I = 0;
				/**
				 * TODO: Change accordingly.
				 * The D constant for the individual motor velocity PIDController of the
				 * rearRight motor.
				 */
				public static final double RR_CONTROLLER_D = 0;
			}
		}
	}
}