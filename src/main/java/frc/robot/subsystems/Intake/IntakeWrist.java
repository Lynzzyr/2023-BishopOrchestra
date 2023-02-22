// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.kIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class IntakeWrist extends PIDSubsystem
{
  private final CANSparkMax motor;
  private final DutyCycleEncoder encoder;

  private final ShuffleboardTab tab_intake;
  private final GenericEntry kP, kI, kD, encPos;

  public IntakeWrist()
  {
    super(new PIDController(kIntake.kWristP, kIntake.kWristI, kIntake.kWristD));

    motor = new CANSparkMax(kIntake.id_motWrist, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    //TODO: Set a current limit
    motor.burnFlash();

    encoder = new DutyCycleEncoder(kIntake.chnl_encWrist);

    getController().setTolerance(0.1);

    tab_intake = Shuffleboard.getTab("Intake");
    kP = tab_intake.add("kWristP", kIntake.kWristP).getEntry();
    kI = tab_intake.add("kWristI", kIntake.kWristI).getEntry();
    kD = tab_intake.add("kWristD", kIntake.kWristD).getEntry();
    encPos = tab_intake.add("Wrist Abs Pos", getMeasurement()).getEntry();
  }

  @Override
  public void useOutput(double output, double setpoint)
  {

    if (output > kIntake.kVoltageLimits.kWristVoltageLimit)
    {
      motor.setVoltage(kIntake.kVoltageLimits.kWristVoltageLimit);
    }
    else if (output < -kIntake.kVoltageLimits.kWristVoltageLimit)
    {
      motor.setVoltage(-kIntake.kVoltageLimits.kWristVoltageLimit);
    }
    else
    {
      motor.setVoltage(output);
    }
  }
  @Override
  public double getMeasurement()
  {
    return getWristPos();
  }

  public double getWristPos()
  {
    double currPos = encoder.getAbsolutePosition();

    if (currPos < 0.25)
    {
      return currPos + 1;
    }
    else
    {
      return currPos;
    }
  }

  public void wristControl(double speed)
  {
    motor.setVoltage(12 * speed);
  }

  @Override
  public void periodic()
  {
    super.periodic();
    encPos.setDouble(getWristPos());
  }
}
