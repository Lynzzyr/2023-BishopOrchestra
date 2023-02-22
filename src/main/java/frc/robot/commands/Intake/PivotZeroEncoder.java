package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.IntakePivot;

public class PivotZeroEncoder extends CommandBase
{
  private final IntakePivot sys_intakePivot;

  public PivotZeroEncoder(IntakePivot subsystem)
  {
    sys_intakePivot = subsystem;

    addRequirements(sys_intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    sys_intakePivot.zeroEncoder();
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