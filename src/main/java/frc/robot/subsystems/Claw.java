package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kClaw;

public class Claw extends SubsystemBase {

    private final WPI_TalonFX clawMot;

    private final TimeOfFlight clawSensor;

    private final DutyCycleEncoder clawDutyEncoder;

    private ShuffleboardTab clawTab;
    private GenericEntry encoderPosEntry, encoderVeloEntry, tempEntry, distanceEntry, isStalledEntry, dutyEncoderEntry;

    private final boolean debug = true;

    public Claw() {
        clawMot = new WPI_TalonFX(kClaw.clawCANID, Constants.kCANBus.bus_rio);

        configMot();

        clawDutyEncoder = new DutyCycleEncoder(kClaw.dutyCycleChannel);

        clawSensor = new TimeOfFlight(kClaw.ToFCANID);

        clawSensor.setRangingMode(RangingMode.Short, 20);

        if (debug) {
            clawTab = Shuffleboard.getTab("Claw");

            encoderPosEntry = clawTab.add("Encoder", getEncoderPosition()).getEntry();
            encoderVeloEntry = clawTab.add("Encoder Velo", getEncoderVelocity()).getEntry();
            isStalledEntry = clawTab.add("Is Stalled", isStalled()).getEntry();
            tempEntry = clawTab.add("Motor Temp", getMotorTempature()).getEntry();  
            distanceEntry = clawTab.add("Distance", getDistanceFromClaw()).getEntry(); 
            dutyEncoderEntry = clawTab.add("Duty", getDutyPosition()).getEntry();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (debug) {
            encoderPosEntry.setDouble(getEncoderPosition());
            encoderVeloEntry.setDouble(getEncoderVelocity());
            isStalledEntry.setBoolean(isStalled());
            tempEntry.setDouble(getMotorTempature());
            distanceEntry.setDouble(getDistanceFromClaw());
            dutyEncoderEntry.setDouble(getDutyPosition());
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

    /**
     * Configures the motor (NeutralMode, Current Limiting, PIDF values)
     * 
     */

    public void configMot() {
        clawMot.configFactoryDefault();

        clawMot.setNeutralMode(NeutralMode.Brake);

        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.enable = true;
        config.currentLimit = kClaw.currentLimit;
        clawMot.configSupplyCurrentLimit(config);

        setPIDF(kClaw.kP, kClaw.kI, kClaw.kD, kClaw.kF);

    }

    /**
     * Sets the Neutral Mode
     * @param newMode Brake, Coast
     */

    public void setNeutralMode(NeutralMode newMode) {
        clawMot.setNeutralMode(newMode);
    }

    /**
     * Sets the PIDF values on the motr
     * @param p proportional
     * @param i intergral
     * @param d derivative
     * @param f feed foward
     */

    public void setPIDF(double p, double i, double d, double f) {
        clawMot.config_kP(0, p);
        clawMot.config_kI(0, i);
        clawMot.config_kD(0, d);
        clawMot.config_kF(0, f);
    }

    /**
     * Stops the motor spin
     */

    public void stopMot() {
        clawMot.set(0);
    }

    /**
     * Opens the claw
     */

    public void openClaw() {
        clawMot.set(ControlMode.Position, kClaw.openPosition);
    }

    /**
     * Closes the claw
     */

    public void closeClaw() {
        clawMot.set(ControlMode.Position, kClaw.coneClosePosition);
    }

    /**
     * Set the claw position to
     * @param pos position
     */

    public void clawGoTo(double pos) {
        clawMot.set(ControlMode.Position, pos);
    }

    /**
     * Get the encoder position
     * @return position
     */

    public double getEncoderPosition() {
        return clawMot.getSelectedSensorPosition();
    }

    /**
     * Get the encoder absolute velocity
     * @return Absolute velocity
     */

    public double getEncoderVelocity() {
        return Math.abs(clawMot.getSelectedSensorVelocity());
    }

    /**
     * Zero the encoder
     */

    public void zeroEncoder() {
        clawMot.setSelectedSensorPosition((0.54 - getDutyPosition()) * kClaw.dutyCycleRatio);
        // clawMot.setSelectedSensorPosition(0);
    }

    /**
     * Spin the claw motor at
     * @param speed % speed (-1, 1)
     */

    public void spinAt(double speed) {
        clawMot.set(speed);
    }

    /**
     * Check if the motor has stalled
     * @return stalled?
     */

    public boolean isStalled() {
        double velo = getEncoderVelocity();
        if (clawMot.getSupplyCurrent() >= 0.1) {
            if (velo <= 20) {
                //motor stalled
                return true;
            } else {
                //motor not stalled
                return false;
            }

        } else {
            //motor not running
            return false;
        }
    }

    /**
     * Get the motor tempature
     * @return tempature
     */

    public double getMotorTempature() {
        return clawMot.getTemperature();
    }

    /**
     * Gets the time of flight distance
     * @return distance in mm
     */

    public double getDistanceFromClaw() {
        return clawSensor.getRange();
    }

    /**
     * Gets the duty cycle encoders position
     * @return position
     */

    public double getDutyPosition() {
        return clawDutyEncoder.getAbsolutePosition();
    }

    /**
     * Resets the duty cycle encoder
     */

    public void resetDutyEncoder() {
        clawDutyEncoder.reset();
    }

}