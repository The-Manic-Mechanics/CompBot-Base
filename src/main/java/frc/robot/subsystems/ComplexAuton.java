// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Auton.PIDControllers.HolonomicController;
/**
 * PathPlanner implementation auton
 */
public class ComplexAuton extends SubsystemBase {
  SendableChooser<Command> autoRoutineChooser;
  /**
    * The auton holonomic controller for -+use with following PathWeaver trajectories
    */
  HolonomicDriveController holoController;
  /** Creates a new ComplexAuton. */
  public ComplexAuton() {
    holoController = new HolonomicDriveController(
        // Correction along the field X axis
        new PIDController(HolonomicController.XCONTROLLER_P, HolonomicController.XCONTROLLER_I, HolonomicController.XCONTROLLER_D),
        // Correction along the field Y axis 
        new PIDController(HolonomicController.YCONTROLLER_P, HolonomicController.YCONTROLLER_I, HolonomicController.YCONTROLLER_D),
        // For rotation correction 
        new ProfiledPIDController(HolonomicController.THETACONTROLLER_P, HolonomicController.THETACONTROLLER_I, HolonomicController.THETACONTROLLER_D,
        new TrapezoidProfile.Constraints(Auton.MAX_SPEED, Auton.MAX_ACCEL))
    );
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
