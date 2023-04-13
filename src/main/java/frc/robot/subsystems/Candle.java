package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANdle;
import frc.robot.Constants.kCANdle.AnimationTypes;
import frc.robot.Constants.kCANdle.LEDColorType;
import frc.robot.Constants.kCANdle.kColors;

public class Candle extends SubsystemBase {

    private final CANdle candle;

    private ColorFlowAnimation colorAnimation;

    private int currentAnimationSlot = -1;

    private int[] LEDOff = {0, 0, 0};

    private int timer = 0;
    private double animationTime = 0;

    private int LEDColors[][] = new int[kCANdle.kConfig.LEDCount][3];
    private int lastLEDColors[][] = new int[kCANdle.kConfig.LEDCount][3];

    private int currentChargeLocation = 0;
    private int maxCharge = 0;

    private LEDColorType currentColor;

    public Candle() {
        candle = new CANdle(kCANdle.kConfig.CANID);

        candle.configFactoryDefault();

        candle.configLEDType(LEDStripType.GRB);

        candle.animate(null, 0);

        setColor(0, 0, 0);

        configBrightness(1);

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
      // candle.setLEDs(0, 0, 0, 0, 0, 8);
      // candle.setLEDs(r, g, b, 0, 8, kCANdle.kConfig.LEDCount);
      for (int i = 0; i < kCANdle.kConfig.LEDCount; i++) {
        setArrayLEDs(i, r, g, b);
        lastLEDColors[i][0] = r;
        lastLEDColors[i][1] = g;
        lastLEDColors[i][2] = b;
      }
      candle.setLEDs(r, g, b);
    }

    /**
     * Sets the current animation playing and clears the LEDs
     * @param animationType The animation you want to play
     * @param r : 0 - 255
     * @param g : Green 0 - 255
     * @param b : Blue 0  - 255
     * @param type the cube or cone type
     */

     public void setAnimation(AnimationTypes animationType, int r, int g, int b, LEDColorType type) {
      currentColor = type;
      setAnimation(animationType, r, g, b);
    }

    /**
     * Sets the current animation playing and clears the LEDs
     * @param animationType The animation you want to play
     * @param r : 0 - 255
     * @param g : Green 0 - 255
     * @param b : Blue 0  - 255
     */

    public void setAnimation(AnimationTypes animationType, int r, int g, int b) {
      // System.out.println(animationType);
        candle.animate(null, currentAnimationSlot);
        switch(animationType) {
            case Static:
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
              maxCharge = 0;
              currentChargeLocation = 0;
              currentAnimationSlot = 4;
              LEDOff[0] = r;
              LEDOff[1] = g;
              LEDOff[2] = b;
              break;
            case EStopped:
              currentAnimationSlot = 5;
              LEDOff[0] = r;
              LEDOff[1] = g;
              LEDOff[2] = b;
              break;
            case EndGame:
              currentAnimationSlot = 6;
              animationTime = 0;
              LEDOff[0] = r;
              LEDOff[1] = g;
              LEDOff[2] = b;
              break;
        }
    }

    /**
     * Turn on LED
     * @param index at index
     * @param brightness brightness of LED
     */

    public void LEDTurnOnAt(int index) {
      // candle.setLEDs(kColors.idle[0], kColors.idle[1], kColors.idle[2], 0, index, 1);
      setArrayLEDs(index, kColors.idle[0], kColors.idle[1], kColors.idle[2]);
    }

    /**
     * Turn off LED
     * @param index at index
     */

    public void LEDTurnOffAt(int index) {
      // candle.setLEDs(LEDOff[0], LEDOff[1], LEDOff[2], 0, index, 1);
      setArrayLEDs(index, LEDOff[0], LEDOff[1], LEDOff[2]);
    }

    /**
     * Turn o LnEDs
     * @param index at index
     * @param count count to turn on
     */

    public void LEDTurnOn(int index, int count) {
      // candle.setLEDs(kColors.idle[0], kColors.idle[1], kColors.idle[2], 0, index, count);
      for (int i = index; i < index + count; i++) {
        setArrayLEDs(i, kColors.idle[0], kColors.idle[1], kColors.idle[2]);
      }
    }

    /**
     * Turn off LEDs
     * @param index at index
     * @param count count to turn off
     */

    public void LEDTurnOff(int index, int count) {
      // candle.setLEDs(LEDOff[0], LEDOff[1], LEDOff[2], 0, index, count);
      for (int i = index; i < index + count; i++) {
        setArrayLEDs(i, LEDOff[0], LEDOff[1], LEDOff[2]);
      }
    }

    /**
     * Turn on LEDs
     * @param index at index
     * @param count count to turn on
     * @param MIN minimum value
     * @param MAX maximum value
     */

    public void LEDTurnOn(int index, int count, int MIN, int MAX) {
      if (MAX >= kCANdle.kConfig.LEDCount) {
        MAX = kCANdle.kConfig.LEDCount - 1;
      }
      for (int i = index; i < index + count; i++) {
        if (i < MAX && i > MIN) {
          // candle.setLEDs(kColors.idle[0], kColors.idle[1], kColors.idle[2], 0, i, 1);
          for (int j = index; j < index + count; j++) {
            setArrayLEDs(j, kColors.idle[0], kColors.idle[1], kColors.idle[2]);
          }
        }
      }
    }

    /**
     * Turn off LEDs
     * @param index at index
     * @param count count to turn off
     * @param MIN minimum value
     * @param MAX maximum value
     */
    
    public void LEDTurnOff(int index, int count, int MIN, int MAX) {
      if (MAX >= kCANdle.kConfig.LEDCount) {
        MAX = kCANdle.kConfig.LEDCount - 1;
      }
      for (int i = index; i < index + count; i++) {
        if (i < MAX && i > MIN) {
          // candle.setLEDs(LEDOff[0], LEDOff[1], LEDOff[2], 0, i, 1);
          for (int j = index; j < index + count; j++) {
            // setArrayLEDs(j, LEDOff);
              setArrayLEDs(j, LEDOff[0], LEDOff[1], LEDOff[2]);
          }
        }
      }
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      timer++;
      if (currentAnimationSlot == -1) {
        for (int i = 0; i < 4; i++) {
          candle.animate(null, i);
        }
      } else if (currentAnimationSlot == 2) {
        //Sin Flow
        animationTime += kCANdle.kColors.kSpeed;
        for (int i = -kCANdle.kColors.LEDSinCount; i < kCANdle.kConfig.LEDCount + kCANdle.kColors.LEDSinCount * 2; i += kCANdle.kColors.LEDSinCount) {
          if (Math.abs(i % (kCANdle.kColors.LEDSinCount * 2)) == 0) {
            LEDTurnOn((int) (i + Math.floor(animationTime) % (kCANdle.kColors.LEDSinCount * 2)), kCANdle.kColors.LEDSinCount);
          } else {
            LEDTurnOff((int) (i + Math.floor(animationTime) % (kCANdle.kColors.LEDSinCount * 2)), kCANdle.kColors.LEDSinCount);
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
            LEDTurnOn((int) (i + animationTime), kCANdle.kColors.LEDSinCount, 7, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft);
          } else {
            LEDTurnOff((int) (i + animationTime), kCANdle.kColors.LEDSinCount, 7, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft);
          }
        }
       
        animationTime *= -1;
       
        for (int i = kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft - kCANdle.kColors.LEDSinCount * 4; i < kCANdle.kConfig.LEDCount + kCANdle.kColors.LEDSinCount * 2; i += kCANdle.kColors.LEDSinCount) {
          if (Math.abs(i % (kCANdle.kColors.LEDSinCount * 2)) <= kCANdle.kColors.LEDSinCount) {
            LEDTurnOff((int) (i + animationTime), kCANdle.kColors.LEDSinCount, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft - 1, kCANdle.kConfig.LEDCount);
          } else {
            LEDTurnOn((int) (i + animationTime), kCANdle.kColors.LEDSinCount, kCANdle.kConfig.LEDCount - kCANdle.kConfig.LEDInnerLeft - 1, kCANdle.kConfig.LEDCount);
          }
        }
      } else if (currentAnimationSlot == 4) {
        //Charged Up
        for (int i = 0; i < 2; i++) {
          LEDTurnOffAt((kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 9) - currentChargeLocation);
          LEDTurnOffAt(kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 2 + currentChargeLocation);
          if ((kCANdle.kConfig.LEDOutter + 9) - currentChargeLocation > 8) {
            LEDTurnOffAt((kCANdle.kConfig.LEDOutter + 6) - currentChargeLocation);
            LEDTurnOffAt((kCANdle.kConfig.LEDOutter * 2 + 9) - currentChargeLocation);
          }
          currentChargeLocation++;
          LEDTurnOnAt((kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 9) - currentChargeLocation);
          LEDTurnOnAt(kCANdle.kConfig.LEDOutter * 2 + kCANdle.kConfig.LEDInnerRight + 2 + currentChargeLocation);
          if ((kCANdle.kConfig.LEDOutter + 9) - currentChargeLocation > 8) {
            LEDTurnOnAt((kCANdle.kConfig.LEDOutter + 6) - currentChargeLocation);
            LEDTurnOnAt((kCANdle.kConfig.LEDOutter * 2 + 9) - currentChargeLocation);
          }
          if (currentChargeLocation == kCANdle.kConfig.LEDInnerLeft - maxCharge + 1) {
            currentChargeLocation = 1;
            maxCharge++;
          } else if (currentChargeLocation > kCANdle.kConfig.LEDInnerLeft) {
            maxCharge = 0;
            currentChargeLocation = 1;
            // candle.setLEDs(0, 0, 0, 0, 8, kCANdle.kConfig.LEDCount);
            setColor(0, 0, 0);
          }
        }
      } else if (currentAnimationSlot == 5) {
        //E Stopped
        if (timer % 10 <= 5) {
          setColor(255, 0, 0);
        } else {
          setColor(LEDOff[0], LEDOff[1], LEDOff[2]);
        }
      } else if (currentAnimationSlot == 6) {
        //End Game
        animationTime++;
        if (animationTime % 5 < 3) {
          setColor(kCANdle.kColors.idle[0], kCANdle.kColors.idle[1], kCANdle.kColors.idle[2]);
        } else {
          setColor(LEDOff[0], LEDOff[1], LEDOff[2]);
        }
        if (animationTime >= 140) {
          idleAnimation();
        }
      }

      //Setting LED colors
      for (int i = 8; i < kCANdle.kConfig.LEDCount; i++) {
        if (LEDColors[i][0] != lastLEDColors[i][0] || LEDColors[i][1] != lastLEDColors[i][1] || LEDColors[i][2] != lastLEDColors[i][2]) {
          candle.setLEDs(LEDColors[i][0], LEDColors[i][1], LEDColors[i][2], 0, i, 1);
        }
        lastLEDColors[i][0] = LEDColors[i][0];
        lastLEDColors[i][1] = LEDColors[i][1];
        lastLEDColors[i][2] = LEDColors[i][2];
      }
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


    public int[] getRGBColors() {
      int[] rgb = {
        lastLEDColors[9][0],
        lastLEDColors[9][1],
        lastLEDColors[9][2]
      };
      return rgb;
    }

    public LEDColorType getLEDType() {
      return currentColor;
    }

    /**
     * @param index at array index
     * @param r red value
     * @param g green value
     * @param b blue value
     */

    public void setArrayLEDs(int index, int r, int g, int b) {
      if (index < kCANdle.kConfig.LEDCount) {
        LEDColors[index][0] = r;
        LEDColors[index][1] = g;
        LEDColors[index][2] = b;
      }
    }

    /**
     * Setting the animation to the Custom Idle animation before the match
     */

    public void idleAnimation() {
      Alliance currentAlliance = DriverStation.getAlliance();
      if (!DriverStation.isDSAttached()) {
        setAnimation(AnimationTypes.SinWave, 0, 0, 0);
      } else if (currentAlliance == Alliance.Red) {
        setAnimation(AnimationTypes.SinWave, 200, 0, 0);
      } else if (currentAlliance == Alliance.Blue) {
        setAnimation(AnimationTypes.SinWave, 0, 0, 255);
      }
    }

    public void FMSAnimation() {
      setAnimation(AnimationTypes.Static, 0, 0, 0);
    }

    /**
     * Setting the animation to the built in animation during game
     */

     public void inGameAnimation() {
      // setAnimation(AnimationTypes.ColorFlow, kCANdle.kColors.idle[0], kCANdle.kColors.idle[1], kCANdle.kColors.idle[2]);
      // setAnimation(AnimationTypes.Static, kCANdle.kColors.cone[0], kCANdle.kColors.cone[1], kCANdle.kColors.cone[2]);
      if (DriverStation.getAlliance() == Alliance.Red) {
        setAnimation(AnimationTypes.Static, 255, 0, 0);
      } else {
        setAnimation(AnimationTypes.Static, 0, 0, 255);
      }
    }

    /**
     * Setting LEDs to the charged up animation
     */

    public void chargedUp() {
      setAnimation(AnimationTypes.ChargedUp, 0, 0, 0);
    }

    /**
     * Sets the animation to the E Stopped animation
     */

    public void EStopped() {
      setAnimation(AnimationTypes.EStopped, 0, 0, 0);
    }

    public void endGame() {
      setAnimation(AnimationTypes.EndGame, 0, 0, 0);
    }

}