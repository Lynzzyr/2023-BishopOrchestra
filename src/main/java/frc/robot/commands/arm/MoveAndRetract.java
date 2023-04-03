// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.Telescope;

public class MoveAndRetract extends CommandBase {
  private final ArmPIDSubsystem sys_arm;
  private double armSetpoint;
  private final Telescope sys_telescope;
  /** Creates a new MoveAndRetract. */
  public MoveAndRetract(ArmPIDSubsystem armPIDSubsystem, double armSetpoint,Telescope telescope) {
    sys_arm = armPIDSubsystem;
    sys_telescope = telescope;
    this.armSetpoint = armSetpoint;

    addRequirements(sys_arm, sys_telescope);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_telescope.extend(Constants.kTelescope.kDestinations.kRetracted);
    sys_telescope.setPrevPos(Constants.kTelescope.kDestinations.kRetracted);
    sys_arm.setSetpoint(armSetpoint);
    sys_arm.enable();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

