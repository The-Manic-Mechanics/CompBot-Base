// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Solenoids;

public class TelescoperOut extends CommandBase {
  /** Creates a new DriveMecanum. */
  // #TODO# Figure out how to combine Arm and Solenoids subsystems into one for command usage
  private final Solenoids sysSolenoids;
  private final Arm sysArm;

  public TelescoperOut(Solenoids inSysSolenoids, Arm inSysArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysSolenoids = inSysSolenoids;
    sysArm = inSysArm;

    addRequirements(sysSolenoids, sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (sysArm.GetArmEnc() < ) {
      sysSolenoids.ToggleTelescope(Value.kReverse);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysSolenoids.ToggleTelescope(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sysSolenoids.GetTelescope() == Value.kReverse) {
    return true;
    } else {
      return false;
    }
  }
}