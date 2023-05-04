package frc.robot.commands.auto.task;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.Constants.kClaw;
import frc.robot.commands.LEDs.BlinkLEDs;
import frc.robot.commands.claw.ClawMovement;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Claw;

public class CloseClawInAuto extends SequentialCommandGroup {

    public CloseClawInAuto(Claw sys_claw, Candle sys_LEDs) {

        Command cmdLED = //blinks the LEDs
                Commands.runOnce(
                    () -> sys_LEDs.setAnimation(
                        AnimationTypes.Static,
                        kCANdle.kColors.cone[0],
                        kCANdle.kColors.cone[1],
                        kCANdle.kColors.cone[2]
                    )
                ).alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(0.05),
                        new BlinkLEDs(sys_LEDs, 0, 255, 0, kCANdle.kColors.blinkSpeed, kCANdle.kColors.blinkTime)
                    )
                );

        addCommands(
            new ParallelDeadlineGroup(
                    new ClawMovement(sys_claw, kClaw.coneClosePosition).withTimeout(0.8),
                    cmdLED
                )
        );
    }
    
}
