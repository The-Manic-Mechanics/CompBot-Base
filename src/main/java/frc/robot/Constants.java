// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ComplexAuton;
import java.io.IOException;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	// FIXME: Javadoc isn't appearing as intended
	public static class Controllers {
		/**
		 * TODO: Change accordingly.
		 * The port which the first driver's controller is connected to.
		 */
		public static final int
            DRIVERONE_PORT = 0,
            /**
			 * TODO: Change accordingly.
             * The port which the second driver's controller is connected to.
             */
            DRIVERTWO_PORT = 1;
	}
	
	public static class LimelightMounting {
	    public static final double
	         /**
			  * TODO: Change accordingly.
              * How many degrees the limelight is mounted from perfectly vertical.
              */
            limelightMountAngleDegrees = 90,
			
            /**
			 * TODO: Change accordingly.
             * The amount of inches from the center of the LimeLight lens to the floor.
             */
			lensHeightInches = 20;
	}

	public static class Gyroscope {
		public static final Port gyroPort = SPI.Port.kMXP;
	}

	public static class AprilTagCoords {
		/**
		 * Function to get the coordinates of a given apriltag as a Translation2D (x and y)
		 * @param in_id The id of the apriltag
		 * @return The field coordinates of the given apriltag as a Translation2D
		 */
		public static Translation2d TranslationFromAprilTagCoordinate(int in_id) {
			int id = in_id;
			return new Translation2d(MAP[id - 1][0], MAP[id - 1][1]);
		}

		/**
		 * TODO: Change accordingly.
		 * X, Y, Z, Heading (In that order)
		 */
		public static final double[][] MAP = {
				{
					593.68, 9.68, 53.38, 120
				},
				{
					637.21, 34.79, 53.38, 120
				},
				{
					652.73, 196.17, 57.13, 180
				},
				{
					652.73, 218.42, 57.13, 180
				},
				{
					578.77, 323.0, 53.38, 270
				},
				{
					72.5, 323.0, 53.38, 270
				},
				{
					-1.5, 218.42, 57.13, 0
				},
				{
					-1.5, 196.17, 57.13, 0
				},
				{
					14.02, 34.79, 53.38, 60
				},
				{
					57.54, 9.68, 53.38, 60
				},
				{
					468.69, 146.19, 52.0, 300
				},
				{
					468.69, 177.1, 52.0, 60
				},
				{
					441.74, 161.62, 52.0, 180
				},
				{
					209.48, 161.62, 52.0, 0
				},
				{
					182.73, 177.1, 52.0, 120
				},
				{
					182.73, 146.19, 52.0, 240
				}
		};
	}

	public static class DriveTrain {
		public static class MotorPorts {
			/**
			 * Front Left CAN Port
			 */
			public static final int
			FRONT_LEFT = 2,
			/**
			 * Front Right CAN Port
			 */
			FRONT_RIGHT = 7,
			/**
			 * Back Left CAN Port
			 */
			BACK_LEFT = 5,
			/**
			 * Back Right CAN Port
			 */
			BACK_RIGHT = 6;
		}

		public static class EncoderPorts {
			/**
			 * Front Left Encoder Channel A
			 */
			public static final int
			FRONT_LEFT_A = 4,
			/**
			 * Front Left Encoder Channel B
			 */
			FRONT_LEFT_B = 5,
			/**
			 * Front Right Encoder Channel A
			 */
			FRONT_RIGHT_A = 6,
			/**
			 * Front Right Encoder Channel B
			 */
			FRONT_RIGHT_B = 7,
			/**
			 * Back Left Encoder Channel A
			 */
			BACK_LEFT_A = 0,
			/**
			 * Back Left Encoder Channel B
			 */
			BACK_LEFT_B = 1,
			/**
			 * Back Right Encoder Channel A
			 */
			BACK_RIGHT_A = 2,
			/**
			 * Back Right Encoder Channel B
			 */
			BACK_RIGHT_B = 3;
		}

		public static class MotorLocations {
			/**
			 * TODO: Change accordingly.
			 * Where the front left wheel is located relative to the center of the robot.
			 */
			public static final double
			FRONT_LEFT = 0.0,
			/**
			 * TODO: Change accordingly.
			 * Where the front right wheel is located relative to the center of the robot.
			 */
			FRONT_RIGHT = 0.0,
			/**
			 * TODO: Change accordingly.
			 * Where the back left  wheel is located relative to the center of the robot in metres.
			 */
			BACK_LEFT = 0.0,
			/**
			 * TODO: Change accordingly.
			 * Where the back right wheel is located relative to the center of the robot in metres.
			 */
			BACK_RIGHT = 0.0;
		}
	}


	public static class Auton {
		// TODO: Fill in these values
		/**z
		 * The max forward speed of the robot in meters per second.
		 */
		public static final double
		MAX_SPEED = .3, 
		/**
		 * The prefered velocity of the robot in autonomous mode in meters per second.
		 */
		DRIVE_VEL = 8.5,
		/**
		 * TODO: Change accordingly.
		 * The distance between the left and right wheels in metres.
		 */
		TRACK_WIDTH_METRES = .4826,
		/**
		 * The distance (In feet) per each pulse on the encoder
		 */
		DISTANCE_PER_PULSE = 25.132741228718 / 8192,
		/**
		 * The maximum acceleration of the robot in meters per second
		 */
		MAX_ACCEL = 0;

		public static final java.util.HashMap<String, Command> EVENT_MAP = new java.util.HashMap<>();

		/**
		 * The file paths to all Pathweaver paths in the project
		 */
		public static final String [] ALL_PATHS = {"paths/DriveStraight.wpilib.json"};

		/**
		 * All paths in the project loaded as usable Trajectories
		 */
		public static Trajectory [] trajectories; {
		try {
			trajectories = ComplexAuton.loadPaths(Auton.ALL_PATHS);
		} catch (IOException ex) {
			DriverStation.reportError("Failed to load trajectories", ex.getStackTrace());
		}
		}

		public static class PIDControllers {
			public static class Holonomic {
				public static final double
				/**
				 * The P constant for the holonomicController's X (Field Relative) correction PID loop
				 */
				XCONTROLLER_P = 1,
				/**
				 * The I constant for the holonomicController's X (Field Relative) correction PID loop
				 */
				XCONTROLLER_I = 0,
				/**
				 * The D constant for the holonomicController's X (Field Relative) correction PID loop
				 */
				XCONTROLLER_D = 0,

				/**
				 * The P constant for the holonomicController's Y (Field Relative) correction PID loop
				 */
				YCONTROLLER_P = 1,
				/**
				 * The I constant for the holonomicController's Y (Field Relative) correction PID loop
				 */
				YCONTROLLER_I = 0,
				/**
				 * The D constant for the holonomicController's Y (Field Relative) correction PID loop
				 */
				YCONTROLLER_D = 0,

				/**
				 * The P constant for the holonomicController's rotation correction PID loop (In degrees)
				 */
				THETACONTROLLER_P = 1,
				/**
				 * The I constant for the holonomicController's rotation correction PID loop (In degrees)
				 */
				THETACONTROLLER_I = 0,
				/**
				 * The D constant for the holonomicController's rotation correction PID loop (In degrees)
				 */
				THETACONTROLLER_D = 0;

			}

			public static class WheelVelocities {
				public static final double
				/**
				 * The P constant for the individual motor velocity PID controller of the frontLeft motor
				 */
				FL_CONTROLLER_P = 1,
				/**
				 * The I constant for the individual motor velocity PID controller of the frontLeft motor
				 */
				FL_CONTROLLER_I = 0,
				/**
				 * The D constant for the individual motor velocity PID controller of the frontLeft motor
				 */
				FL_CONTROLLER_D = 0,

				/**
				 * The P constant for the individual motor velocity PID controller of the frontRight motor
				 */
				FR_CONTROLLER_P = 1,
				/**
				 * The I constant for the individual motor velocity PID controller of the frontRight motor
				 */
				FR_CONTROLLER_I = 0,
				/**
				 * The D constant for the individual motor velocity PID controller of the frontRight motor
				 */
				FR_CONTROLLER_D = 0,

				/**
				 * The P constant for the individual motor velocity PID controller of the rearLeft motor
				 */
				RL_CONTROLLER_P = 1,
				/**
				 * The I constant for the individual motor velocity PID controller of the rearLeft motor
				 */
				RL_CONTROLLER_I = 0,
				/**
				 * The D constant for the individual motor velocity PID controller of the rearLeft motor
				 */
				RL_CONTROLLER_D = 0,

				/**
				 * The P constant for the individual motor velocity PID controller of the rearRight motor
				 */
				RR_CONTROLLER_P = 1,
				/**
				 * The I constant for the individual motor velocity PID controller of the rearRight motor
				 */
				RR_CONTROLLER_I = 0,
				/**
				 * The D constant for the individual motor velocity PID controller of the rearRight motor
				 */
				RR_CONTROLLER_D = 0;
				
			}
		}
	}
}