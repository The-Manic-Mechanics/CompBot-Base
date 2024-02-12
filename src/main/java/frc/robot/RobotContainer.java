// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Auton.PIDControllers.Holonomic;
import frc.robot.Constants.Auton.PIDControllers.WheelVelocities;
import frc.robot.commands.ClankAuton;
import frc.robot.commands.DriveMecanum;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ComplexAuton;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Solenoids;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // -----------------------------------------
  // Subsystems
  // -----------------------------------------

  private final Solenoids sysSolenoids = new Solenoids();
  private final Gyroscope sysGyroscope = new Gyroscope();
  private final DriveTrain sysDriveTrain = new DriveTrain();
  private final ComplexAuton sysComplexAuton = new ComplexAuton();

  // --------------------------------------------------------------------

  // -----------------------------------------
  // Command
  // -----------------------------------------

  private final DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);
  private final ClankAuton cmdClankAuton = new ClankAuton(sysDriveTrain, sysComplexAuton);

  // ---------------------------------------------------------------------

  // -----------------------------------------
  // Controllers
  // -----------------------------------------

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final XboxController driverOneController = new XboxController(Controllers.DRIVERONE_PORT);
  public static final JoystickButton 
    mainButton1 = new JoystickButton(driverOneController, 1),
    mainButton2 = new JoystickButton(driverOneController, 4);

  public static final XboxController driverTwoController = new XboxController(Controllers.DRIVERTWO_PORT);
  public static final JoystickButton 
    driverSecondA = new JoystickButton(driverTwoController, 1),
    driverSecondY = new JoystickButton(driverTwoController, 4),
    driverSecondLeftBump = new JoystickButton(driverTwoController, 5),
    driverSecondRghtBump = new JoystickButton(driverTwoController, 6);
  // ---------------------------------------------------------------------------------

  // --------------------------------------------
  // Auton controller
  // --------------------------------------------

  /**
   * Controller for following autonomous paths with mecanum, why didn't these fuckers tell me about this.  
   */
  MecanumControllerCommand mecanumController;

  // ----------------------------------------------------------------------------------

  // --------------------------------------------
  // SmartDashboard
  // --------------------------------------------

  public static SendableChooser<Trajectory> autonPathChooser;

  public static Pose2d initPose;

  // ----------------------------------------------------------------------------------
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // The robot's subsystems and commands are defined here...
    sysDriveTrain.setDefaultCommand(cmdDriveMecanum);

    // Configure the trigger bindings
    configureBindings();

    // -------------------------
    // SmartDashboard
    // -------------------------

    autonPathChooser = new SendableChooser<Trajectory>();
    autonPathChooser.addOption("None", null);
    Constants.Auton.LoadTrajectoriesFromPaths();
    autonPathChooser.setDefaultOption("Drive Straight", Auton.trajectories[0]);
  
    SmartDashboard.putData(autonPathChooser);

		initPose = new Pose2d(autonPathChooser.getSelected().getInitialPose().getX(), autonPathChooser.getSelected().getInitialPose().getY(), Gyroscope.sensor.getRotation2d());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Configure controller bindings here.
    // mainButton1.onTrue(cmdMove);
    // mainButton2.onTrue(cmdBrakeUp);

    // The DriveMecanum command periodically checks the controller's joysticks for acceleration.
  }

  public Command getAutonomousCommand() { 
   mecanumController = new MecanumControllerCommand(
      autonPathChooser.getSelected(), 
      ComplexAuton.getPoseDual(), 
      ComplexAuton.feedforward, 
      DriveTrain.Kinematics.mecanumDriveKinematics,
      // X controller (Field Space)
      new PIDController(Holonomic.XCONTROLLER_P, Holonomic.XCONTROLLER_I, Holonomic.XCONTROLLER_D), 
      // Y controller (Field Space)
      new PIDController(Holonomic.YCONTROLLER_P, Holonomic.YCONTROLLER_I, Holonomic.YCONTROLLER_D),  
      new ProfiledPIDController(Holonomic.THETACONTROLLER_P, Holonomic.THETACONTROLLER_I, Holonomic.THETACONTROLLER_D,
        new TrapezoidProfile.Constraints(Auton.MAX_SPEED, Auton.MAX_ACCEL)), 
      Auton.MAX_SPEED, 
      new PIDController(WheelVelocities.FL_CONTROLLER_P, WheelVelocities.FL_CONTROLLER_I, WheelVelocities.FL_CONTROLLER_D), 
      new PIDController(WheelVelocities.RL_CONTROLLER_P, WheelVelocities.RL_CONTROLLER_I, WheelVelocities.RL_CONTROLLER_D), 
      new PIDController(WheelVelocities.FR_CONTROLLER_P, WheelVelocities.FR_CONTROLLER_I, WheelVelocities.FR_CONTROLLER_D), 
      new PIDController(WheelVelocities.RR_CONTROLLER_P, WheelVelocities.RR_CONTROLLER_I, WheelVelocities.RR_CONTROLLER_D), 
      DriveTrain.Kinematics.getWheelSpeeds(), 
      DriveTrain.Kinematics::driveVolts, 
      sysDriveTrain, sysComplexAuton);


    return Commands.sequence(
        new InstantCommand(() -> {
          DriveTrain.Odometry.resetDriveOdometry(autonPathChooser.getSelected().getInitialPose());
          DriveTrain.Motors.frontLeft.setSafetyEnabled(false);
          DriveTrain.Motors.frontRight.setSafetyEnabled(false);
          DriveTrain.Motors.rearLeft.setSafetyEnabled(false);
          DriveTrain.Motors.rearRight.setSafetyEnabled(false);
        }),
        mecanumController,
        new InstantCommand(() -> {
          DriveTrain.mecanum.driveCartesian(0, 0, 0);
          DriveTrain.Motors.frontLeft.setSafetyEnabled(true);
          DriveTrain.Motors.frontRight.setSafetyEnabled(true);
          DriveTrain.Motors.rearLeft.setSafetyEnabled(true);
          DriveTrain.Motors.rearRight.setSafetyEnabled(true);
        }));
  }
}
