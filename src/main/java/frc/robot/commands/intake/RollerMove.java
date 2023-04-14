// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeRoller;

public class RollerMove extends CommandBase
{
  private final IntakeRoller sys_intakeRoller;
  private double voltage;

  public RollerMove(IntakeRoller subsystem, double voltage)
  {
    sys_intakeRoller = subsystem;
    this.voltage = voltage;

    addRequirements(sys_intakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    sys_intakeRoller.rollerControl(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    sys_intakeRoller.rollerControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
