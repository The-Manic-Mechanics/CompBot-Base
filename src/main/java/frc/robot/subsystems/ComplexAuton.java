// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Auton.PIDControllers.HolonomicController;
import frc.robot.subsystems.DriveTrain.Odometry;
/**
 * PathPlanner implementation auton
 */
public class ComplexAuton extends SubsystemBase {
  SendableChooser<Command> autoRoutineChooser;
  public static PathPlannerPath path;
  public static Command followPathCommand;
  /** Creates a new ComplexAuton. */
  public ComplexAuton() {
    AutoBuilder.configureHolonomic(
      // Robot pose supplier, currently using the odometry
      DriveTrain.Odometry.mecanumDriveOdometry::getPoseMeters,
      // Method to reset odometry (Only called if auto has a starting pose)
      Odometry.resetDriveOdometry(),
      // Robot speed supplier, taken as a ChassisSpeeds (Robot relative)
      DriveTrain.Kinematics.getMecanumChassisSpeeds(),
      // Method that drives the robot via a ChassisSpeeds
      this::driveRobotRelative, 
        new HolonomicPathFollowerConfig(
          // Translation PID constants
            new PIDConstants(
              HolonomicController.TRANSCONTROLLER_P, 
              HolonomicController.TRANSCONTROLLER_I, 
              HolonomicController.TRANSCONTROLLER_D),
          // Rotation PID constants
            new PIDConstants(
              HolonomicController.ROTCONTROLLER_P, 
              HolonomicController.ROTCONTROLLER_I, 
              HolonomicController.ROTCONTROLLER_D), 
          // Max speed of one motor
            Auton.MAX_METRES_PER_SEC,
          // Distance from robot center to furthest motor (Meters) 
            Auton.TRACK_WIDTH_METRES / 2,
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

    // Will do nothing as default
    autoRoutineChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auton Chooser", autoRoutineChooser);
    // return autoRoutineChooser.getSelected();
    path = PathPlannerPath.fromPathFile("Test");

    followPathCommand = AutoBuilder.followPath(ComplexAuton.path);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
      // FIXME TODO: So what goes here? The configureHolonomic function doc says this function should "set the robot's robot-relative chassis speeds"
      DriveTrain.mecanum.driveCartesian(speeds.vxMetersPerSecond /* multiplied by something to make it fit within the limits, loop?, etc... */, speeds.vyMetersPerSecond, Math.toDegrees(speeds.omegaRadiansPerSecond));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
