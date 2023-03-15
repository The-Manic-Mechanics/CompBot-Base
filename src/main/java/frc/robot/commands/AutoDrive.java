// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
  DriveTrain sysDriveTrain;
  /** Creates a new AutoDrive. */
  public AutoDrive(DriveTrain inSysDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    addRequirements(sysDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // sysDriveTrain.followPathEvents(sysDriveTrain.autoRoutineChooser.getSelected());

    // MecanumControllerCommand mecanumControllerCommand = 
    // new MecanumControllerCommand(
    //   null,
    //   sysDriveTrain.mecanumDriveOdometry :: getPoseMeters, 
    //   new SimpleMotorFeedforward(0, 0, 0), 
    //   sysDriveTrain.mecanumDriveKinematics, 
    //   new PIDController(.05, 0, 0), 
    //   new PIDController(.05, 0, 0), 
    //   new ProfiledPIDController(.05, 0, 0, new TrapezoidProfile.Constraints(Autonomous.MAX_METRES_PER_SEC, Autonomous.MAX_ACCEL)), 
    //   Autonomous.MAX_METRES_PER_SEC, 
    //   new PIDController(.5, 0, 0),
    //   new PIDController(.5, 0, 0), 
    //   new PIDController(.5, 0, 0), 
    //   new PIDController(.5, 0, 0), 
    //   sysDriveTrain :: getCurMecWheelSpeeds, 
    //   sysDriveTrain :: setDriveMotsVolts,
    //   sysDriveTrain);

//      // Create a voltage constraint to ensure we don't accelerate too fast

//      var autoVoltageConstraint =

//      new DifferentialDriveVoltageConstraint(

//          new SimpleMotorFeedforward(

//              Autonomous.VOLTS,

//              Autonomous.VOLT_SECS_PER_M,

//              Autonomous.VOLT_SECS_SQURED_PER_M),

//          sysDriveTrain.mecanumDriveKinematics,

//          10);


//  // Create config for trajectory

//  TrajectoryConfig config =

//      new TrajectoryConfig(

//              Autonomous.MAX_METRES_PER_SEC,

//              Autonomous.MAX_ACCEL)

//          // Add kinematics to ensure max speed is actually obeyed

//          .setKinematics(sysDriveTrain.mecanumDriveKinematics)

//          // Apply the voltage constraint

//          .addConstraint(autoVoltageConstraint);


//  // An example trajectory to follow.  All units in meters.

//  Trajectory exampleTrajectory =

//      TrajectoryGenerator.generateTrajectory(

//          // Start at the origin facing the +X direction

//          new Pose2d(0, 0, new Rotation2d(0)),

//          // Pass through these two interior waypoints, making an 's' curve path

//          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

//          // End 3 meters straight ahead of where we started, facing forward

//          new Pose2d(3, 0, new Rotation2d(0)),

//          // Pass config

//          config);


//  RamseteCommand ramseteCommand =

//      new RamseteCommand(

//          exampleTrajectory,

//          sysDriveTrain.mecanumDriveOdometry::getPose,

//          new RamseteController(Autonomous.RAMSETE_B, Autonomous.RAMSETE_ZETA),

//          new SimpleMotorFeedforward(

//              Autonomous.VOLTS,

//              Autonomous.VOLT_SECS_PER_M,

//              Autonomous.VOLT_SECS_SQURED_PER_M),

//          sysDriveTrain.mecanumDriveKinematics,

//          sysDriveTrain::getWheelSpeeds,

//          new PIDController(Autonomous.DRIVE_VEL, 0, 0),

//          new PIDController(Autonomous.DRIVE_VEL, 0, 0),

//          // RamseteCommand passes volts to the callback

//          sysDriveTrain::mecanumDriveVolts,

//          sysDriveTrain);


//  // Reset odometry to the starting pose of the trajectory.

//  m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());


//  // Run path following command, then stop at the end.

//  return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
