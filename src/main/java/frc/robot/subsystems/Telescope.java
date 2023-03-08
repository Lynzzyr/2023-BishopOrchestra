package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {

    private final CANSparkMax mot_extender;

    private final RelativeEncoder s_encoder;
    private double prevPos;

    private final SparkMaxPIDController c_pidController;

    private static final double kP = Constants.kTelescope.kPID.kP;
    private static final double kI = Constants.kTelescope.kPID.kI;
    private static final double kD = Constants.kTelescope.kPID.kD;
    private static final double kF = Constants.kTelescope.kPID.kF;

    private static DigitalInput s_maxLimSwitch;
    private static DigitalInput s_minLimSwitch;

    boolean debugMode = true;
    private static HashMap<String, GenericEntry> shuffleboardFields; 
  

    public Telescope() {
        
        mot_extender = new CANSparkMax(Constants.kTelescope.kDeviceID.MOTOR_CAN_ID, MotorType.kBrushless);
        mot_extender.restoreFactoryDefaults();
        
        mot_extender.setIdleMode(IdleMode.kBrake);
        mot_extender.setInverted(true);
        mot_extender.setSmartCurrentLimit(30);

        s_encoder = mot_extender.getEncoder();
        s_encoder.setPositionConversionFactor(Constants.kTelescope.kSprocket.kGearConversionFactor);
        zeroEncoder();

        c_pidController = mot_extender.getPIDController();
        configPID();
        c_pidController.setOutputRange(-1.25, 1.25);
        mot_extender.burnFlash();

        s_maxLimSwitch = new DigitalInput(Constants.kTelescope.kDeviceID.MAX_LIMIT_SWITCH_ID);
        s_minLimSwitch = new DigitalInput(Constants.kTelescope.kDeviceID.MIN_LIMIT_SWITCH_ID);

        if (debugMode) {
            shuffleboardFields = new HashMap<String, GenericEntry>();

            ShuffleboardLayout limitSwitch = Shuffleboard.getTab("Arm").getLayout("LimSwitch", BuiltInLayouts.kGrid).withSize(2, 1);
            ShuffleboardLayout encoder = Shuffleboard.getTab("Arm").getLayout("EncoderLayout", BuiltInLayouts.kList);
            ShuffleboardLayout speed = Shuffleboard.getTab("Arm").getLayout("VelocityArm", BuiltInLayouts.kGrid);

            shuffleboardFields.put("LimSwitchMax", limitSwitch.add("Max Limit Switch", getMaxLimSwitch()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0,0).getEntry());
            shuffleboardFields.put("LimSwitchMin", limitSwitch.add("Min Limit Switch", getMinLimSwitch()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 0).getEntry());
            
            shuffleboardFields.put("EncoderData", encoder.add("Encoder Data", getDistance()).withWidget(BuiltInWidgets.kTextView).getEntry());
        
            shuffleboardFields.put("SpeedOfArm", speed.add("Speed of Extension", rotationDirection()).withWidget(BuiltInWidgets.kTextView).getEntry());
        }
    }

    public void zeroEncoder() {
        s_encoder.setPosition(0);
    }

    public double getDistance() {
        return s_encoder.getPosition();
    }

    public void configPID() {
        c_pidController.setP(kP);
        c_pidController.setI(kI);
        c_pidController.setD(kD);
        c_pidController.setFF(kF);
    }

    public boolean getMaxLimSwitch() {
        return !s_maxLimSwitch.get();
    }
    
    public boolean getMinLimSwitch() {
        return !s_minLimSwitch.get();
    }

    public int rotationDirection() {
        
        if (s_encoder.getVelocity() > 0.5) {
            return 1;
        } else if (s_encoder.getVelocity() < -0.5) {
            return -1;
        }
        return 0;
    }
    
    public boolean getSwitch() {
        if (rotationDirection() == 1) {
            return getMaxLimSwitch();
        } else if (rotationDirection() == -1) {
            return getMinLimSwitch();
        } else {
            return false;
        }
    }
    
    public void extend(double setpoint) {
        c_pidController.setReference(setpoint, ControlType.kPosition);
    }

    public void stopExtending() {
        mot_extender.disable();
    }

    public void moveToZero() {
        mot_extender.set(0.1);
    }


    public double getPrevPos() {
        return prevPos;
    }

    public void setPrevPos(double prevPos) {
        this.prevPos = prevPos;
    }

    /**
     * Sets the Neutral Mode
     * @param newMode Brake, Coast
     */

     public void setNeutralMode(IdleMode newMode) {
        mot_extender.setIdleMode(newMode);
    }

    @Override
    public void periodic() {
        if (debugMode) {
            shuffleboardFields.get("LimSwitchMax").setBoolean(getMaxLimSwitch());
            shuffleboardFields.get("LimSwitchMin").setBoolean(getMinLimSwitch());

            shuffleboardFields.get("EncoderData").setDouble(getDistance());

            shuffleboardFields.get("SpeedOfArm").setDouble(rotationDirection());
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}