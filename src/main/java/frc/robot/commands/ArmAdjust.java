// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPIDSubsystem;

public class ArmAdjust extends CommandBase {
  private final ArmPIDSubsystem sys_arm;
  private double setpoint;


  /** Creates a new ArmRotation2. */
  public ArmAdjust(ArmPIDSubsystem armPIDSubsystem) {
    sys_arm = armPIDSubsystem;
    this.setpoint = sys_arm.getController().getSetpoint() + 0.01;

    addRequirements(sys_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_arm.setSetpoint(setpoint);
    sys_arm.enable();
  
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    sys_arm.disable();
    sys_arm.setPrevPos(setpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_arm.getController().atSetpoint();
  }
}