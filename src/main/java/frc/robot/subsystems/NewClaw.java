// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.kCANBus;
import frc.robot.Constants.kClaw;
import frc.robot.Constants.kClaw.kClawState;

public class NewClaw extends PIDSubsystem {
  private final WPI_TalonFX clawMot;

    private final DutyCycleEncoder clawDutyEncoder;

    private final TimeOfFlight s_tofLeft;
    private final TimeOfFlight s_tofRight;

    private kClawState currentState = kClawState.kOpen;


    private ShuffleboardTab clawTab;

    private GenericEntry tempEntry, dutyEncoderEntry, tofLeftAvgEntry, tofLeftRealEntry, tofRightAvgEntry, tofRightRealEntry;

    private final boolean debug = true;

    private final double[] lastToFValues = new double[5];
    private int indexCount;
  
  /** Creates a new NewClaw. */
  public NewClaw() {
    super(new PIDController(Constants.kClaw.kP, Constants.kClaw.kI, Constants.kClaw.kD));
    clawMot = new WPI_TalonFX(kClaw.clawCANID, kCANBus.bus_rio);
    getController().setTolerance(0);

    configMot();

    s_tofLeft = new TimeOfFlight(36);
    s_tofLeft.setRangingMode(RangingMode.Short, s_tofLeft.getSampleTime());
    s_tofRight = new TimeOfFlight(37);
    s_tofRight.setRangingMode(RangingMode.Short, s_tofLeft.getSampleTime());

    clawDutyEncoder = new DutyCycleEncoder(kClaw.dutyCycleChannel);

    if (debug) {
      clawTab = Shuffleboard.getTab("Claw");
      tempEntry = clawTab.add("Motor Temp", getMotorTempature()).getEntry();  
      dutyEncoderEntry = clawTab.add("Duty", getDutyPosition()).getEntry();
      tofLeftAvgEntry = clawTab.add("Time Of Flight (Left Avg)", rollingToFAvg(s_tofLeft)).getEntry();
      tofLeftRealEntry = clawTab.add("Time Of Flight (Left Real)", getDistanceToFLeft()).getEntry();
      tofRightAvgEntry = clawTab.add("Time Of Flight (Right Avg)", rollingToFAvg(s_tofRight)).getEntry();
      tofRightRealEntry = clawTab.add("Time Of Flight (Right Real)", getDistanceToFRight()).getEntry();
    }
  }

  public void configMot() {
    clawMot.configFactoryDefault();

    clawMot.setNeutralMode(NeutralMode.Brake);

    SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
    config.enable = true;
    config.currentLimit = kClaw.currentLimit;
    clawMot.configSupplyCurrentLimit(config);
    clawMot.setInverted(true);

    setPID(kClaw.kP, kClaw.kI, kClaw.kD);

}

public void setSpeed(double speed) {
  clawMot.set(speed);
}

public void stopMotor() {
  clawMot.set(0);
}

public void setPID(double p, double i, double d) {
  m_controller.setP(p);
  m_controller.setI(i);
  m_controller.setD(d);
}


  @Override
  public void useOutput(double output, double setpoint) {
    if (output >= kClaw.outputLimit) {
      output = kClaw.outputLimit;
    } else if (output <= -kClaw.outputLimit) {
      output = -kClaw.outputLimit;
    }
    clawMot.setVoltage(output*12);
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    return getDutyPosition();

    // Return the process variable measurement here
  }

  /**
   * Return the tempature of the motor
   * @return c
   */

  public double getMotorTempature() {
    return clawMot.getTemperature();
  }

  /**
   * Gets the duty cycle encoder absolute position
   * @return -1, 1
   */

  public double getDutyPosition() {
    return clawDutyEncoder.getAbsolutePosition();
  }

  public TimeOfFlight getLeftToF() {
    return s_tofLeft;
  }

  public TimeOfFlight getRightToF() {
    return s_tofRight;
  }

  public double getDistanceToFLeft() {
    return s_tofLeft.getRange();
  }

  public double getDistanceToFRight() {
    return s_tofRight.getRange();
  }

  public double rollingToFAvg(TimeOfFlight s_tof) {
    int currentDist = (int) s_tof.getRange();

    if (indexCount > 1) {
      indexCount = 0;
    }

    if (currentDist > 2) {
      lastToFValues[indexCount] = currentDist;
    }
    
    int sum = 0;
    for (double value : lastToFValues) {
      sum += value;
    }
    
    int avg = sum/2;

    if (avg > 300) {
      avg = 300;
    }

    indexCount++;
    return avg;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (debug) {
      tempEntry.setDouble(getMotorTempature());
      dutyEncoderEntry.setDouble(getMeasurement());
      tofLeftAvgEntry.setDouble(rollingToFAvg(s_tofLeft));
      tofLeftRealEntry.setDouble(getDistanceToFLeft());
      tofRightAvgEntry.setDouble(rollingToFAvg(s_tofRight));
      tofRightRealEntry.setDouble(getDistanceToFRight());
    }



  }
}
