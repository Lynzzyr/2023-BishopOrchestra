package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Auto extends SequentialCommandGroup {

    public Auto(Drivetrain sys_drivetrain, Trajectory trajectory) {
        super(
            new AutoPathPlanning(sys_drivetrain, trajectory),
            new BalancingChargeStation(sys_drivetrain),
            new TurnDegrees(sys_drivetrain, -90)
        );
    }

}