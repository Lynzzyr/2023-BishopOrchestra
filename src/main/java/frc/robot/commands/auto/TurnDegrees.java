package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kTurnDegrees;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends PIDCommand {

    private final Drivetrain m_drivetrain;

    private static final double kP = kTurnDegrees.kP_chargeStation;
    private static final double kI = kTurnDegrees.kI_chargeStation;
    private static final double kD = kTurnDegrees.kD_chargeStation;

    // Shuffleboard
    // private final ShuffleboardTab sb_turningTab;
    // private final GenericEntry nt_kP, nt_kI, nt_kD;

    public TurnDegrees(Drivetrain drivetrain, double degrees) {
        super(
            new PIDController(kP, kI, kD),
            drivetrain::getHeading,
            degrees,
            output -> drivetrain.arcadeDrive(0, output),
            drivetrain
        );

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        m_drivetrain = drivetrain;

        // Add items to Shuffleboard
        // sb_turningTab = Shuffleboard.getTab("Turn degrees");
        // nt_kP = sb_turningTab.add("kP", 0).getEntry();
        // nt_kI = sb_turningTab.add("kI", 0).getEntry();
        // nt_kD = sb_turningTab.add("kD", 0).getEntry();

        getController().enableContinuousInput(-kTurnDegrees.maxAngle, kTurnDegrees.maxAngle);
        getController().setTolerance(kTurnDegrees.angleTolerance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.resetGyro();

        // Update PID values from Shuffleboard
        // getController().setP(nt_kP.getDouble(0));
        // getController().setI(nt_kI.getDouble(0));
        // getController().setD(nt_kD.getDouble(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}