// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.kIntake.kSetpoints.kPivotSetpoints;
import frc.robot.Constants.kIntake.kSetpoints.kWristSetpoints;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeWrist;

public class IntakePickupSequence extends ParallelCommandGroup {
  public IntakePickupSequence(IntakePivot pivot, IntakeWrist wrist, IntakeRoller roller) {
    addCommands(
      new PivotMove(pivot, kPivotSetpoints.kPivotExtended),
      new WristMove(wrist, kWristSetpoints.kWristPickup),
      new RollerMove(roller, 3.6)
    );
  }
}
