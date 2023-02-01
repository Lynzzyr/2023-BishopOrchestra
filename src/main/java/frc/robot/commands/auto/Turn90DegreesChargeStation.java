package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kTurn90DegreesChargeStation;
import frc.robot.subsystems.Drivetrain;

public class Turn90DegreesChargeStation extends PIDCommand {

    public static enum TurnDirection {
        LEFT, RIGHT
    }

    private final Drivetrain m_drivetrain;

    private static final double kP = kTurn90DegreesChargeStation.kP_chargeStation;
    private static final double kI = kTurn90DegreesChargeStation.kI_chargeStation;
    private static final double kD = kTurn90DegreesChargeStation.kD_chargeStation;

    // Shuffleboard
    // private final ShuffleboardTab sb_turningTab;
    // private final GenericEntry nt_kP, nt_kI, nt_kD;

    /**
     * This command should only be used to turn 90 degrees on the charge station.
     * 
     * The PID values are tuned for the charge station surface,
     * and for turning 90 degrees (either direction) specifically.
     * 
     * @param drivetrain
     * @param degrees
     */
    public Turn90DegreesChargeStation(Drivetrain drivetrain, TurnDirection direction) {
        super(
            new PIDController(kP, kI, kD),
            drivetrain::getHeading,
            (direction == TurnDirection.RIGHT) ? 90 : -90, // turn 90 degrees if right, turn -90 degrees if left
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

        getController().enableContinuousInput(-kTurn90DegreesChargeStation.maxAngle, kTurn90DegreesChargeStation.maxAngle);
        getController().setTolerance(kTurn90DegreesChargeStation.angleTolerance);
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