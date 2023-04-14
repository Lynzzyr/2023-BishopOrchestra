package frc.robot.commands.auto.task;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kBalancing;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;

public class BalancingChargeStation extends PIDCommand {

    private final Drivetrain m_drivetrain;
    private final Candle m_candle;

    private boolean isBalanced = false;

    private static double kP = kBalancing.kP;
    private static double kI = kBalancing.kI;
    private static double kD = kBalancing.kD;

    // Shuffleboard
    boolean debugMode = false; // DO NOT ENABLE: currently causes errors; TODO: Fix debug mode
    private ShuffleboardTab sb_balancingTab;
    private GenericEntry nt_kP, nt_kI, nt_kD;

    public BalancingChargeStation(Drivetrain drivetrain, Candle candle) {
        super(
            new PIDController(kP, kI, kD),
            drivetrain::getPitch,
            kBalancing.targetPitch,
            output -> drivetrain.arcadeDrive(output, 0),
            drivetrain
        );

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain, candle);

        m_drivetrain = drivetrain;
        m_candle = candle;

        // Add items to Shuffleboard
        if (debugMode) {
            sb_balancingTab = Shuffleboard.getTab("Balancing");
            nt_kP = sb_balancingTab.add("kP", 0).getEntry();
            nt_kI = sb_balancingTab.add("kI", 0).getEntry();
            nt_kD = sb_balancingTab.add("kD", 0).getEntry();
        }

        getController().enableContinuousInput(-kBalancing.maxAngle, kBalancing.maxAngle);
        getController().setTolerance(kBalancing.angleTolerance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Update PID values from Shuffleboard
        m_candle.setAnimation(AnimationTypes.Static, 255, 0, 0);
        if (debugMode) {
            getController().setP(nt_kP.getDouble(0));
            getController().setI(nt_kI.getDouble(0));
            getController().setD(nt_kD.getDouble(0));
        }
    }

    @Override
    public void execute() {
        super.execute();
        if (getController().atSetpoint()) {
            if (!isBalanced) {
                m_candle.setAnimation(AnimationTypes.Static, 0, 255, 0);
                isBalanced = true;
            }
        } else {
            if (isBalanced) {
                m_candle.setAnimation(AnimationTypes.Static, 255, 0, 0);
                isBalanced = false;
            }
        }
    }

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