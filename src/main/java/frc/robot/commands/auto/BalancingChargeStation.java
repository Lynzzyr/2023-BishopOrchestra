package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kBalancing;
import frc.robot.subsystems.Drivetrain;

public class BalancingChargeStation extends PIDCommand {

    private final Drivetrain m_drivetrain;

    private static final double kP = kBalancing.kP;
    private static final double kI = kBalancing.kI;
    private static final double kD = kBalancing.kD;

    // Shuffleboard
    // private final ShuffleboardTab sb_balancingTab;
    // private final GenericEntry nt_kP, nt_kI, nt_kD;

    public BalancingChargeStation(Drivetrain drivetrain) {
        super(
            new PIDController(kP, kI, kD),
            drivetrain::getPitch,
            kBalancing.targetPitch,
            output -> drivetrain.arcadeDrive(output, 0),
            drivetrain
        );

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        m_drivetrain = drivetrain;

        // Add items to Shuffleboard
        // sb_balancingTab = Shuffleboard.getTab("Balancing");
        // nt_kP = sb_balancingTab.add("kP", 0).getEntry();
        // nt_kI = sb_balancingTab.add("kI", 0).getEntry();
        // nt_kD = sb_balancingTab.add("kD", 0).getEntry();

        getController().enableContinuousInput(-kBalancing.maxAngle, kBalancing.maxAngle);
        getController().setTolerance(kBalancing.angleTolerance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Update PID values from Shuffleboard
        // getController().setP(nt_kP.getDouble(0));
        // getController().setI(nt_kI.getDouble(0));
        // getController().setD(nt_kD.getDouble(0));
    }

    // @Override
    // public void end(boolean interrupted) {
    //     super.end(interrupted);

    //     if (!interrupted) {
    //         Commands.runOnce(() -> new BalancingChargeStation(m_drivetrain));
    //     }
    // }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    // // Returns true when the command should end.
    // @Override
    // public boolean isFinished() {
    //     return getController().atSetpoint() && Math.abs(m_drivetrain.getLeftVelocity()) < 0.01;
    // }

}