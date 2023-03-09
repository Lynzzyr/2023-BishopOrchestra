// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.IntakePivot;

public class PivotManualMove extends CommandBase
{
  private final IntakePivot pivot;
  private double voltage;

  public PivotManualMove(IntakePivot subsystem, double voltage)
  {
    pivot = subsystem;
    this.voltage = voltage;

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    pivot.pivotControl(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    pivot.pivotControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
