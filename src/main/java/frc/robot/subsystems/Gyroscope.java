package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyroscope extends SubsystemBase {
	/**
	 * The sensor used to obtain data.
	 */
	public static AHRS sensor;
	/**
	 * The current pitch rotation of the robot.
	 */
	public static double pitch,
			/**
			 * The current roll rotation of the robot.
			 */
			roll,
			/**
			 * The current yaw rotation of the robot.
			 */
			yaw;

	public Gyroscope() {
		// Initialize the sensor.
		sensor = new AHRS(Constants.Gyroscope.gyroPort);
	}

	@Override
	public void periodic() {
		// Poll data from the sensor and store it.
		pitch = sensor.getPitch();
		roll = sensor.getRoll();
		yaw = sensor.getYaw();

		// Uncomment to make debugging values available on the SmartDashboard.

		// SmartDashboard.putNumber("Pitch", pitch);
		// SmartDashboard.putNumber("Roll", roll);
		// SmartDashboard.putNumber("Yaw", yaw);
		// SmartDashboard.putNumber("Accel X", sensor.getWorldLinearAccelX());
		// SmartDashboard.putNumber("Accel Y", sensor.getWorldLinearAccelY());
		// SmartDashboard.putNumber("Accel Z", sensor.getWorldLinearAccelZ());
	}
}
