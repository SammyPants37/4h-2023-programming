// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.claw;

public final class Autos {
  public static CommandBase balance(drive drive, double backDistance) {
    return Commands.sequence(
            drive.driveToDistance(backDistance),
            drive.balance());
  }

  public static CommandBase scoreCubeStay(elevator elevator, arm arm, claw claw) {
    return Commands.sequence(
            elevator.goToHeight(83),
            arm.goToLength(56),
            Commands.parallel(claw.rightGoToPosition(0.075), claw.leftGoToPosition(0.075)),
            moveArmIn(elevator, arm));
  }

  public static CommandBase scoreCube(elevator elevator, arm arm, claw claw) {
    return Commands.sequence(
            elevator.goToHeight(83),
            arm.goToLength(56),
            Commands.parallel(claw.rightGoToPosition(0.075), claw.leftGoToPosition(0.075)));
  }

  public static CommandBase moveArmIn(elevator elevator, arm arm) {
    return Commands.sequence(
            arm.goToLength(1),
            elevator.goToHeight(1));
  }


  public static CommandBase scoreCubeBalance(elevator elevator, arm arm, claw claw, drive drive,
                                             double backDistance, double forwardDistance) {
    return Commands.sequence(
            scoreCube(elevator, arm, claw),
            moveArmIn(elevator, arm),
            drive.driveToDistance(backDistance),
            balance(drive, forwardDistance));
  }


  public static CommandBase scoreCubeLeave(drive drive, elevator elevator, arm arm, claw claw,
                                           double distance) {
    return Commands.sequence(
            scoreCube(elevator, arm, claw),
            moveArmIn(elevator, arm),
            drive.driveToDistance(distance));
  }


  private Autos() {
    // not meant to be defined
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
