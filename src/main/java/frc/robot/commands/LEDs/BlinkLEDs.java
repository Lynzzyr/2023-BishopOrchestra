package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.Constants.kCANdle.LEDColorType;
import frc.robot.subsystems.Candle;

public class BlinkLEDs extends CommandBase {

    private final Candle m_candle;
    private final int r;
    private final int g;
    private final int b;
    private final int blinkSpeed;
    private final int blinkCount;

    private LEDColorType currentType;
    private int timer;

    public BlinkLEDs(Candle candle, int r, int g, int b, int blinkSpeed, int blinkCount) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_candle = candle;
        this.r = r;
        this.g = g;
        this.b = b;
        this.blinkSpeed = blinkSpeed;
        this.blinkCount = blinkCount;

        addRequirements(m_candle);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer = 0;
        currentType = m_candle.getLEDType();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        timer++;
        if (timer % blinkSpeed < blinkSpeed / 2) {
          m_candle.setAnimation(AnimationTypes.Static, r, g, b);
        } else {
          switchToDefaultColor();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        switchToDefaultColor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.floor(timer / blinkSpeed) >= blinkCount && blinkCount != -1;
    }

    public void switchToDefaultColor() {
        if (currentType == LEDColorType.Cube) {
          m_candle.setAnimation(
            AnimationTypes.Static,
            kCANdle.kColors.cube[0],
            kCANdle.kColors.cube[1],
            kCANdle.kColors.cube[2]
          );
        } else {
          m_candle.setAnimation(
            AnimationTypes.Static,
            kCANdle.kColors.cone[0],
            kCANdle.kColors.cone[1],
            kCANdle.kColors.cone[2]
          );
        }
      }

}
