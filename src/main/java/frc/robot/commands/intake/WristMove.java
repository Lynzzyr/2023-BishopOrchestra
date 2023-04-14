// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeWrist;

public class WristMove extends CommandBase
{
  private final IntakeWrist sys_intakeWrist;
  private double setpoint;
  
  public WristMove(IntakeWrist subsystem, double setpoint)
  {
    sys_intakeWrist = subsystem;
    this.setpoint = setpoint;

    addRequirements(sys_intakeWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    sys_intakeWrist.setSetpoint(setpoint);
    sys_intakeWrist.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
