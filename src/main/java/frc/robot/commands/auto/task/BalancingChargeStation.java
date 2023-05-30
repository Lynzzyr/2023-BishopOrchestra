package frc.robot.commands.auto.task;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kBalancing;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Drivetrain;

public class BalancingChargeStation extends PIDCommand {

    private final Candle m_candle;

    private boolean isBalanced = false;

    private static double kP = kBalancing.kP;
    private static double kI = kBalancing.kI;
    private static double kD = kBalancing.kD;

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

        m_candle = candle;

        getController().enableContinuousInput(-kBalancing.maxAngle, kBalancing.maxAngle);
        getController().setTolerance(kBalancing.angleTolerance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_candle.setAnimation(AnimationTypes.Static, 255, 0, 0);
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