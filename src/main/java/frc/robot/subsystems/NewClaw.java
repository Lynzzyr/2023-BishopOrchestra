// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

    private kClawState currentState = kClawState.kOpen;

    private ShuffleboardTab clawTab;

    private GenericEntry tempEntry, dutyEncoderEntry;

    private final boolean debug = true;
  
  /** Creates a new NewClaw. */
  public NewClaw() {
    super(new PIDController(Constants.kClaw.kP, Constants.kClaw.kI, Constants.kClaw.kD));
    clawMot = new WPI_TalonFX(kClaw.clawCANID, kCANBus.bus_rio);
    getController().setTolerance(0);

    configMot();

    clawDutyEncoder = new DutyCycleEncoder(kClaw.dutyCycleChannel);

    if (debug) {
      clawTab = Shuffleboard.getTab("Claw");
      tempEntry = clawTab.add("Motor Temp", getMotorTempature()).getEntry();  
      dutyEncoderEntry = clawTab.add("Duty", getDutyPosition()).getEntry();
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

  @Override
  public void periodic() {
    super.periodic();
    if (debug) {
      tempEntry.setDouble(getMotorTempature());
      dutyEncoderEntry.setDouble(getMeasurement());
    }
  }
}