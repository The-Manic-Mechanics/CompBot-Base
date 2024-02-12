// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain.Motors;

public class SysId extends SubsystemBase {
  SysIdRoutine routine;
  /** Creates a new SysId. */
  public SysId(DriveTrain sysDriveTrain) {
    // Creates a SysIdRoutine
  routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(this::driveVoltage, null, sysDriveTrain)
);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public void driveVoltage(Measure<Voltage> driveVolts) {
    Motors.frontLeft.setVoltage(driveVolts.baseUnitMagnitude());
    Motors.frontRight.setVoltage(driveVolts.baseUnitMagnitude());
    Motors.rearLeft.setVoltage(driveVolts.baseUnitMagnitude());
    Motors.rearRight.setVoltage(driveVolts.baseUnitMagnitude());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.driverTwoController.getAButton() == true)
      CommandScheduler.getInstance().schedule(this.sysIdQuasistatic(Direction.kReverse));

    if (RobotContainer.driverTwoController.getYButton() == true)
      CommandScheduler.getInstance().schedule(this.sysIdQuasistatic(Direction.kForward));

    if (RobotContainer.driverTwoController.getXButton() == true)
      CommandScheduler.getInstance().schedule(this.sysIdDynamic(Direction.kForward));

    if (RobotContainer.driverTwoController.getBButton() == true)
      CommandScheduler.getInstance().schedule(this.sysIdDynamic(Direction.kReverse));
  }
}
