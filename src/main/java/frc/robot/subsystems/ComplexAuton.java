// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Auton.PIDControllers.Holonomic;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * PathPlanner implementation auton
 */
public class ComplexAuton extends SubsystemBase {
  /**
    * The auton holonomic controller for -+use with following PathWeaver trajectories
    */
  HolonomicDriveController holoController;

  public static SimpleMotorFeedforward feedforward;
  
  /** Creates a new ComplexAuton. */
  public ComplexAuton() {
    // FIXME: What does this acutally do?
    holoController = new HolonomicDriveController(
        // Correction along the field X axis
        new PIDController(Holonomic.XCONTROLLER_P, Holonomic.XCONTROLLER_I, Holonomic.XCONTROLLER_D),
        // Correction along the field Y axis 
        new PIDController(Holonomic.YCONTROLLER_P, Holonomic.YCONTROLLER_I, Holonomic.YCONTROLLER_D),
        // For rotation correction 
        new ProfiledPIDController(Holonomic.THETACONTROLLER_P, Holonomic.THETACONTROLLER_I, Holonomic.THETACONTROLLER_D,
        new TrapezoidProfile.Constraints(Auton.MAX_SPEED, Auton.MAX_ACCEL))
    );  

    // TODO: Unsure on what this does exactly but I know we can get the values from SysID 
    feedforward = new SimpleMotorFeedforward(0.1, 0.1, 0.1);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // FIXME: Hacky but it should work, pay attention to this doober
    MecanumDriveWheelSpeeds wheelSpeeds = DriveTrain.Kinematics.mecanumDriveKinematics.toWheelSpeeds(speeds);
    double ySpeed, xSpeed, zSpeed;

    /* If the ChassisSpeeds is telling us to go left then the vyMetersPerSecond should be positive, so we tell the Mecanum Drive controller to drive left at the speed of one of the wheels (They should all be the same).
     * Otherwise we go right.
     */
    if (speeds.vyMetersPerSecond > 0)
      ySpeed = wheelSpeeds.frontLeftMetersPerSecond;
    else
      ySpeed = -wheelSpeeds.frontLeftMetersPerSecond;

    /* If the ChassisSpeeds is telling us to go forward then the vxMetersPerSecond should be positive, so we tell the Mecanum Drive controller to drive forward at the speed of one of the wheels (They should all be the same).
     * Otherwise we go backwards.
     */
    if (speeds.vxMetersPerSecond > 0)
      xSpeed = wheelSpeeds.frontLeftMetersPerSecond;
    else
      xSpeed = -wheelSpeeds.frontLeftMetersPerSecond;

    /* If the ChassisSpeeds is telling us to turn counterclockwise then the omegaRadiansPerSecond should be positive, so we tell the Mecanum Drive controller to turn counterclockwise at the speed of one of the wheels (They should all be the same).
     * Otherwise we go clockwise.
     */
    if (speeds.omegaRadiansPerSecond > 0)
      zSpeed = wheelSpeeds.frontRightMetersPerSecond;
    else
      zSpeed = -wheelSpeeds.frontRightMetersPerSecond;
    
    DriveTrain.mecanum.driveCartesian(xSpeed, ySpeed, zSpeed);
  }

  // FIXME: May overload command execution

  /**
   * Loads the inputted paths from their files into variables
   * @param paths The paths to the trajectories in the form "paths/YourPath.wpilib.json"
   * @return The an array containing the loaded trajectories in the same order that they were loaded
   */
  public static Trajectory[] loadPaths(String[] paths) throws IOException {
    Trajectory[] trajectories = new Trajectory[paths.length];
    for (int i = 0; i != paths.length; i++) {
      try {
        Path f_path = Filesystem.getDeployDirectory().toPath().resolve(paths[i]);
        DriverStation.reportWarning("\n\n\n\nFILE PATH: " + f_path.toString() + "\n\n\n\n\n", true);
        trajectories[i] = TrajectoryUtil.fromPathweaverJson(f_path);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to %open trajectory: " + paths[i], ex.getStackTrace());
        throw ex;
      }
    }
    return trajectories;
  }

  /**
   * Gets the pose of the robot using an apriltag if avalible, and uses odemetry if not
   * @return The bot pose as a Pose2d based off the nearest apriltag, otherwise use the odometry
   */
  public static Supplier<Pose2d> getPoseDual() {
    if (LimeLight.tagID != 0)
      return (Supplier<Pose2d>)() -> LimeLight.getBotPose2d();
      return (Supplier<Pose2d>)() -> DriveTrain.Odometry.mecanumDriveOdometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
