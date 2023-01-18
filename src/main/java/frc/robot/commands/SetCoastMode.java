package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SetCoastMode extends InstantCommand {
    private final Drivetrain m_drivetrain;

    public SetCoastMode(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.setNeutralMode(NeutralMode.Coast);
    }

    // Allow this command to run when disabled
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}