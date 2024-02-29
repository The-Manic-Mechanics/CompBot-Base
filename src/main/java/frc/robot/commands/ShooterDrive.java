// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Controllers.Sax.ButtonPorts;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterDrive extends Command {
  Shooter sysShooter;
  /** Creates a new ShooterDrive. */
  public ShooterDrive(Shooter inSysShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysShooter = inSysShooter;
    addRequirements(sysShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: The drivers may want this changed
    // Turns on the shooter if it is past the threshold and if the Red button is pressed
    if (Intake.Encoders.lift.get() <= Constants.Intake.SHOOTER_ON_LIMIT) 
      if (RobotContainer.saxController.getRawButton(ButtonPorts.RED))
        Shooter.setSpeed(Constants.Shooter.SPEED);
      else
        Shooter.setSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
