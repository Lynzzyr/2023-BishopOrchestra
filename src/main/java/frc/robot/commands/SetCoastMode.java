package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Telescope;

public class SetCoastMode extends InstantCommand {
    private final Drivetrain m_drivetrain;
    private final Claw m_claw;
    private final Telescope m_telescope;

    public SetCoastMode(Drivetrain drivetrain, Claw claw, Telescope telescope) {
        addRequirements(drivetrain, claw, telescope);
        m_claw = claw;
        m_telescope = telescope;
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.setNeutralMode(NeutralMode.Coast);
        m_claw.setNeutralMode(NeutralMode.Coast);
        m_telescope.setNeutralMode(IdleMode.kCoast);
    }

    // Allow this command to run when disabled
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}