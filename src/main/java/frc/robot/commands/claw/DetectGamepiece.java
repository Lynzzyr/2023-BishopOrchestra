package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kClaw;
import frc.robot.subsystems.NewClaw;

public class DetectGamepiece extends CommandBase {

    private final NewClaw sys_claw;
    private int rumbleTime = -1;
    private boolean rumblingDone;
    private CommandXboxController joystickMain;
    private CommandXboxController joystickSecondary;
    private boolean cube;

    public DetectGamepiece(NewClaw subsystem, CommandXboxController joystickMain, CommandXboxController joystickSecondary, boolean cube) {
        sys_claw = subsystem;
        this.cube = cube;
        this.joystickMain = joystickMain;
        this.joystickSecondary = joystickSecondary;
        rumblingDone = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_claw);
        
    }

    public void rumbleController(double value, int time) {
        rumbleTime = time;
        joystickMain.getHID().setRumble(RumbleType.kBothRumble, value);
        joystickSecondary.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void stopRumble() {
        joystickMain.getHID().setRumble(RumbleType.kBothRumble, 0);
        joystickSecondary.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    public void updateRumble() {
        if (rumbleTime == 0) {
            joystickMain.getHID().setRumble(RumbleType.kBothRumble, 0);
            joystickSecondary.getHID().setRumble(RumbleType.kBothRumble, 0);
            rumbleTime = -1;
            rumblingDone = true;
        } else {
            rumbleTime--;
        }
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rumblingDone = false;
        if (cube) {
            if ((sys_claw.getDistanceToFLeft() < kClaw.coneDistanceThreshold || sys_claw.getDistanceToFRight() < kClaw.coneDistanceThreshold) && !sys_claw.getController().atSetpoint()) {
                rumbleController(0.5, 6);
            } else {
                rumblingDone = true;
            }
        } else {
            if ((sys_claw.getDistanceToFLeft() < kClaw.coneDistanceThreshold || sys_claw.getDistanceToFRight() < kClaw.coneDistanceThreshold) || !sys_claw.getController().atSetpoint()) {
                rumbleController(0.3, 6);
            } else {
                rumblingDone = true;
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updateRumble();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        stopRumble();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return rumblingDone;
    }

}