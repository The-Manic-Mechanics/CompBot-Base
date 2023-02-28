// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TelescoperIn;
import frc.robot.commands.TelescoperOut;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GrabbyArm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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

  // ---------------------------
  // Subsystem
  // ---------------------------

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrain sysDriveTrain = new DriveTrain();

  private final GrabbyArm sysArm = new GrabbyArm();

  // ----------------------------------------------------------------------------------

  // ---------------------------
  // Commands
  // ---------------------------

  private final DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);

  private final ClawOpen cmdClawOpen = new ClawOpen(sysArm);
  private final ClawClose cmdClawClose = new ClawClose(sysArm);

  private final TelescoperIn cmdTelescoperIn = new TelescoperIn(sysArm);
  private final TelescoperOut cmdTelescoperOut = new TelescoperOut(sysArm);

  private final ArmDrive cmdArmDrive = new ArmDrive(sysArm);
  // ----------------------------------------------------------------------------------

  // ---------------------------
  // Controller
  // ---------------------------

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final XboxController driverMainController = new XboxController(ControllerConstants.DRIVERONE_PORT);

  public static final XboxController driverSecondController = new XboxController(ControllerConstants.DRIVERTWO_PORT);
  private final JoystickButton driverSecondA = new JoystickButton(driverSecondController, 1);
  private final JoystickButton driverSecondY = new JoystickButton(driverSecondController, 4);
  private final JoystickButton driverSecondLeftBump = new JoystickButton(driverSecondController, 5);
  private final JoystickButton driverSecondRghtBump = new JoystickButton(driverSecondController, 6);
  // ---------------------------------------------------------------------------------


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

  // --------------------------
  // SetDefaultCommand
  // --------------------------

  sysDriveTrain.setDefaultCommand(cmdDriveMecanum);

  // ----------------------------------------------------------------------------------

  
    // Configure the trigger bindings
    configureBindings();

  // -------------------------
  // SmartDashboard
  // -------------------------


  // ----------------------------------------------------------------------------------
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

  // ----------------------------
  // Driver (Main)
  // ----------------------------
    // Assigning driver main button X to cmdAutoBalance


  // -------------------------------------------------------------------------------------

  
  // ----------------------------
  // Driver (Secondary)
  // ----------------------------

    driverSecondA.onTrue(cmdTelescoperIn);
    driverSecondY.onTrue(cmdTelescoperOut);

    driverSecondLeftBump.onTrue(cmdClawOpen);
    driverSecondRghtBump.onTrue(cmdClawClose);

    


  
  // -------------------------------------------------------------------------------------
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
