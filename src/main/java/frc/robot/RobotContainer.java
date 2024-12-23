// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.commands.AutoShooterAlign;
import frc.robot.commands.ClimberDrive;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.ShooterDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ComplexAuton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SysID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
    private static Gyroscope sysGyroscope = new Gyroscope();
    private static DriveTrain sysDriveTrain = new DriveTrain();
    private static ComplexAuton sysComplexAuton = new ComplexAuton(sysDriveTrain);
    private static SysID sysSysID = new SysID(sysDriveTrain);

    public static AutoShooterAlign cmdAutoShooterAlign = new AutoShooterAlign(sysDriveTrain, sysComplexAuton);

    public static Pose2d initPose;

    public RobotContainer() {
        Intake sysIntake = new Intake();
        Shooter sysShooter = new Shooter();
        Climber sysClimber = new Climber();

        ShooterDrive cmdShooterDrive = new ShooterDrive(sysShooter);
        IntakeDrive cmdIntakeDrive = new IntakeDrive(sysIntake);
        DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);
        ClimberDrive cmdClimberDrive = new ClimberDrive(sysClimber);

        sysShooter.setDefaultCommand(cmdShooterDrive);
        sysDriveTrain.setDefaultCommand(cmdDriveMecanum);
        sysIntake.setDefaultCommand(cmdIntakeDrive);
        sysClimber.setDefaultCommand(cmdClimberDrive);

        configureBindings();

        Auton.loadTrajectoriesFromPaths();
    }

    // TODO: Bind SysID commands to gamepad buttons.
    private void configureBindings() {
        // Define the controls used in new functions within HumanInterface.CommandMap and then supply the command here.
        // HumanInterface.CommandMap.runSysIDQuasistaticForwards(sysSysID.runQuasistatic(Direction.kReverse));
        // HumanInterface.CommandMap.runSysIDQuasistaticBackwards(sysSysID.runQuasistatic(Direction.kForward));
        // HumanInterface.CommandMap.runSysIDDynamicForwards(sysSysID.runDynamic(Direction.kReverse));
        // HumanInterface.CommandMap.runSysIDDynamicBackwards(sysSysID.runDynamic(Direction.kForward));
    }

    public Command getAutonomousCommand() {
        // If the "no trajectory" option is chosen, do not run anything.
        if (DriveTrain.Odometry.autonRoutineChooser.getSelected() == null)
            return new InstantCommand(() -> {});

        return DriveTrain.Odometry.autonRoutineChooser.getSelected() instanceof Trajectory ?
                ComplexAuton.createDriveCommand(((Trajectory)DriveTrain.Odometry.autonRoutineChooser.getSelected()), true)
                :
                (Command)DriveTrain.Odometry.autonRoutineChooser.getSelected();
    }
}