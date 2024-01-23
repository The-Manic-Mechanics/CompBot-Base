// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Auton.PIDControllers.HolonomicController;
/**
 * PathPlanner implementation auton
 */
public class ComplexAuton extends SubsystemBase {
  /** Creates a new ComplexAuton. */
  public ComplexAuton() {
    // FIXME Unfinished
    AutoBuilder.configureHolonomic(
                 // Robot pose supplier, currently using the odometry
                DriveTrain.mecanumDriveOdometry::getPoseMeters,
                // TODO Make this method
                // Method to reset odometry (Only called if auto has a starting pose)
                DriveTrain::,
                // Robot speed supplier, taken as a ChassisSpeeds (Robot relative)
                this::getRobotRelativeSpeeds,
                // Method that drives the robot via a ChassisSpeeds
                this::driveRobotRelative, 
                new HolonomicPathFollowerConfig(
                        // TODO Move these to constants
                        // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Rotation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), 
                        // Max speed of one motor
                        4.5,
                        // Distance from robot center to furthest motor (Metres) 
                        0.4,
                        // TODO: Default path replanning config, need to check api for this one
                        new ReplanningConfig()
                ),
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
              },
              this // Reference to this subsystem to set requirements
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
