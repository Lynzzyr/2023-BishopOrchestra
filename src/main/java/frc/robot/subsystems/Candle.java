package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.Constants.kCANdle.kColors;

public class Candle extends SubsystemBase {

    private final CANdle candle;

    private ColorFlowAnimation colorAnimation;

    private int currentAnimationSlot = -1;

    private int[] LEDOff = {0, 0, 0};

    private int timer = 0;
    private double animationTime = 0;

    private int currentChargeLocation = 0;
    private int maxCharge = 0;

    public Candle() {
        candle = new CANdle(kCANdle.kConfig.CANID);

        candle.configLEDType(LEDStripType.RGB);

        candle.animate(null, 0);

        candle.setLEDs(0, 0, 0, 0, 0, kCANdle.kConfig.LEDCount);

    }

    /**
     * Configs the brightness of all the LEDs
     * @param brightness brightness between 0 and 1;
     */

    public void configBrightness(double brightness) {
        candle.configBrightnessScalar(brightness);
    }

    /**
     * Sets all of the LEDs to
     * @param r : Red 0   - 255
     * @param g : Green 0 - 255
     * @param b : Blue 0  - 255
     */

    public void setColor(int r, int g, int b) {
        candle.setLEDs(0, 0, 0, 0, 8, kCANdle.kConfig.LEDCount);
    }

    /**
     * Sets the current animation playing and clears the LEDs
     * @param animationType The animation you want to play
     * @param r : 0 - 255
     * @param g : Green 0 - 255
     * @param b : Blue 0  - 255
     */

    public void setAnimation(AnimationTypes animationType, int r, int g, int b) {
        candle.animate(null, currentAnimationSlot);
        switch(animationType) {
            case Static:
                //TODO: might break if u dont have delay
                currentAnimationSlot = 0;
                setColor(r, g, b);
                break;
            case ColorFlow:
              currentAnimationSlot = 1;
              colorAnimation = new ColorFlowAnimation(r, g, b, 0, kCANdle.kColors.gameSpeed, kCANdle.kConfig.LEDCount, Direction.Forward);
              setColor(0, 0, 0);
              candle.animate(colorAnimation, currentAnimationSlot);
              break;
            case SinFlow:
                currentAnimationSlot = 2;
                LEDOff[0] = r;
                LEDOff[1] = g;
                LEDOff[2] = b;
                break;
            case SinWave:
                currentAnimationSlot = 3;
                LEDOff[0] = r;
                LEDOff[1] = g;
                LEDOff[2] = b;
                break;
            case ChargedUp:
                currentAnimationSlot = 4;
                LEDOff[0] = r;
                LEDOff[1] = g;
                LEDOff[2] = b;
                break;
        }
    }

    /**
     * Map the value using interpolated data
     * @param n number
     * @param start1 starting of input
     * @param stop1 stopping of input
     * @param start2 starting of output
     * @param stop2 stopping of output
     * @param withinBounds within the bounds
     * @return interpolated number
     */

    public double map(double n, double start1, double stop1, double start2, double stop2, boolean withinBounds) {
        double newval = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
        if (!withinBounds) {
          return newval;
        }
        if (start2 < stop2) {
          return constrain(newval, start2, stop2);
        } else {
          return constrain(newval, stop2, start2);
        }
    }

    /**
     * Constraining the value
     * @param n number
     * @param low lowest
     * @param high highest
     * @return constrained number
     */
    
    public double constrain(double n, double low, double high) {
        return Math.max(Math.min(n, high), low);
    }

    /**
     * Turn on LED
     * @param index at index
     * @param brightness brightness of LED
     */

    public void LEDTurnOnAt(int index, double brightness) {
      candle.setLEDs((int) (kColors.idle[0] * brightness), (int) (kColors.idle[1] * brightness), (int) (kColors.idle[2] * brightness), 0, index, 1);
    }

    /**
     * Turn off LED
     * @param index at index
     * @param brightness brightness of LED
     */

    public void LEDTurnOffAt(int index, double brightness) {
      candle.setLEDs((int) (LEDOff[0] * brightness), (int) (LEDOff[1] * brightness), (int) (LEDOff[2] * brightness), 0, index, 1);
    }

    /**
     * Turn o LnEDs
     * @param index at index
     * @param count count to turn on
     * @param brightness brightness of LEDs
     */

    public void LEDTurnOn(int index, int count, double brightness) {
      candle.setLEDs((int) (kColors.idle[0] * brightness), (int) (kColors.idle[1] * brightness), (int) (kColors.idle[2] * brightness), 0, index, count);
    }

    /**
     * Turn off LEDs
     * @param index at index
     * @param count count to turn off
     * @param brightness brightness of LEDs
     */

    public void LEDTurnOff(int index, int count, double brightness) {
      candle.setLEDs((int) (LEDOff[0] * brightness), (int) (LEDOff[1] * brightness), (int) (LEDOff[2] * brightness), 0, index, count);
    }

    /**
     * Turn on LEDs
     * @param index at index
     * @param count count to turn on
     * @param MIN minimum value
     * @param MAX maximum value
     * @param brightness brightness of LEDs
     */

    public void LEDTurnOn(int index, int count, int MIN, int MAX, double brightness) {
      for (int i = index; i < index + count; i++) {
        if (i < MAX && i > MIN) {
          candle.setLEDs((int) (kColors.idle[0] * brightness), (int) (kColors.idle[1] * brightness), (int) (kColors.idle[2] * brightness), 0, index, count);
        }
      }
    }

    /**
     * Turn off LEDs
     * @param index at index
     * @param count count to turn off
     * @param MIN minimum value
     * @param MAX maximum value
     * @param brightness brightness
     */
    
    public void LEDTurnOff(int index, int count, int MIN, int MAX, double brightness) {
      for (int i = index; i < index + count; i++) {
        if (i < MAX && i > MIN) {
          candle.setLEDs((int) (LEDOff[0] * brightness), (int) (LEDOff[1] * brightness), (int) (LEDOff[2] * brightness), 0, index, count);
        }
      }
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      timer++;
      double brightness = map(candle.getTemperature(), 20, 70, 1, 0, true);
      if (currentAnimationSlot == -1) {
        for (int i = 0; i < 4; i++) {
          candle.animate(null, i);
        }
      } else if (currentAnimationSlot == 2) {
        //Sin Flow
        animationTime += kCANdle.kColors.kSpeed;
        for (int i = -kCANdle.kColors.LEDSinCount; i < kCANdle.kConfig.LEDCount + kCANdle.kColors.LEDSinCount * 2; i += kCANdle.kColors.LEDSinCount) {
          if (Math.abs(i % (kCANdle.kColors.LEDSinCount * 2)) == 0) {
            LEDTurnOn((int) (i + Math.floor(animationTime) % (kCANdle.kColors.LEDSinCount * 2)), kCANdle.kColors.LEDSinCount, brightness);
          } else {
            LEDTurnOff((int) (i + Math.floor(animationTime) % (kCANdle.kColors.LEDSinCount * 2)), kCANdle.kColors.LEDSinCount, brightness);
          }
        }   
      } else if (currentAnimationSlot == 3) {
        //Sin Wave
        animationTime = Math.sin(timer * kCANdle.kColors.sinFrequency) * kCANdle.kColors.sinFrequencySpeed;
        
        if (Math.abs(animationTime) >= 19.5) {
          timer += 4;
        }
  
        animationTime = Math.floor(animationTime);
        
        for (int i = -kCANdle.kColors.LEDSinCount * 2; i < kCANdle.kConfig.LEDCount; i += kCANdle.kColors.LEDSinCount) {
          if (Math.abs(i % (kCANdle.kColors.LEDSinCount * 2)) == 0) {
            LEDTurnOn((int) (Math.floor(i + animationTime)), kCANdle.kColors.LEDSinCount, 7, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft, 1);
          } else {
            LEDTurnOff((int) Math.floor(i + animationTime), kCANdle.kColors.LEDSinCount, 7, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft, 1);
          }
        }
        
        animationTime *= -1;
        
        for (int i = kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft - kCANdle.kColors.LEDSinCount * 4; i < kCANdle.kConfig.LEDCount + kCANdle.kColors.LEDSinCount * 2; i += kCANdle.kColors.LEDSinCount) {
          if (Math.abs(i % (kCANdle.kColors.LEDSinCount * 2)) <= kCANdle.kColors.LEDSinCount) {
            LEDTurnOff((int) (Math.floor(i + animationTime)), kCANdle.kColors.LEDSinCount, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft - 1, kCANdle.kConfig.LEDCount, 1);
          } else {
            LEDTurnOn((int) (Math.floor(i + animationTime)), kCANdle.kColors.LEDSinCount, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft - 1, kCANdle.kConfig.LEDCount, 1);
          }
        }
      } else if (currentAnimationSlot == 4) {
        //Charged Up
        for (int i = 0; i < 2; i++) {
          LEDTurnOffAt((kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 9) - currentChargeLocation, 1);
          LEDTurnOffAt(kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 2 + currentChargeLocation, 1);
          if ((kCANdle.kConfig.LEDOutter + 9) - currentChargeLocation > 8) {
            LEDTurnOffAt((kCANdle.kConfig.LEDOutter + 6) - currentChargeLocation, 1);
            LEDTurnOffAt((kCANdle.kConfig.LEDOutter * 2 + 9) - currentChargeLocation, 1);
          }
          currentChargeLocation++;
          LEDTurnOnAt((kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 9) - currentChargeLocation, 1);
          LEDTurnOnAt(kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 2 + currentChargeLocation, 1);
          if ((kCANdle.kConfig.LEDOutter + 9) - currentChargeLocation > 8) {
            LEDTurnOnAt((kCANdle.kConfig.LEDOutter + 6) - currentChargeLocation, 1);
            LEDTurnOnAt((kCANdle.kConfig.LEDOutter * 2 + 9) - currentChargeLocation, 1);
          }
          if (currentChargeLocation == kCANdle.kConfig.LEDInnerLeft - maxCharge + 1) {
            currentChargeLocation = 1;
            maxCharge++;
          } else if (currentChargeLocation > kCANdle.kConfig.LEDInnerLeft) {
            maxCharge = 0;
            currentChargeLocation = 1;
            candle.setLEDs(0, 0, 0, 0, 8, kCANdle.kConfig.LEDCount);
          }
        }
      }
      SmartDashboard.putNumber("CANdle Temp: ", candle.getTemperature());
      SmartDashboard.putNumber("Brightness: ", brightness);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

    /**
     * Gets the current animation
     * @return current animation slot
     */

    public int getCurrentAnimation() {
      return currentAnimationSlot;
    }

    /**
     * Setting the animation to the Custom Idle animation before the match
     */

    public void idleAnimation() {
      if (DriverStation.getAlliance() == Alliance.Red) {
        setAnimation(AnimationTypes.SinFlow, 150, 0, 0);
      } else {
        setAnimation(AnimationTypes.SinFlow, 0, 0, 255);
      }
    }

    /**
     * Setting the animation to the built in animation during game
     */

    public void inGameAnimation() {
      setAnimation(AnimationTypes.ColorFlow, kCANdle.kColors.idle[0], kCANdle.kColors.idle[1], kCANdle.kColors.idle[2]);
      candle.setLEDs(0, 0, 0, 0, 8, kCANdle.kConfig.LEDCount);
    }

    /**
     * Setting LEDs to the charged up animation
     */

    public void chargedUp() {
      setAnimation(AnimationTypes.ChargedUp, 0, 0, 0);
    }

}