// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.kIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class IntakePivot extends PIDSubsystem
{
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final ShuffleboardTab tab_intake;
  private final GenericEntry kP, kI, kD, encPos;

  public IntakePivot()
  {
    super(new PIDController(kIntake.kPivotP, kIntake.kPivotI, kIntake.kPivotD));

    motor = new CANSparkMax(kIntake.id_motPivot, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(kIntake.kCurrentLimits.kPivotCurrentLimit);

    encoder = motor.getEncoder();

    motor.burnFlash();

    tab_intake = Shuffleboard.getTab("Intake");
    kP = tab_intake.add("kPivotP", kIntake.kPivotP).getEntry();
    kI = tab_intake.add("kPivotI", kIntake.kPivotI).getEntry();
    kD = tab_intake.add("kPivotD", kIntake.kPivotD).getEntry();
    encPos = tab_intake.add("Pivot Rel Pos", getMeasurement()).getEntry();
  }

  @Override
  public void useOutput(double output, double setpoint)
  {
    
    if (output > kIntake.kVoltageLimits.kPivotVoltageLimit)
    {
      motor.setVoltage(kIntake.kVoltageLimits.kPivotVoltageLimit);
    }
    else if (output < -kIntake.kVoltageLimits.kPivotVoltageLimit)
    {
      motor.setVoltage(-kIntake.kVoltageLimits.kPivotVoltageLimit);
    }
    else
    {
      motor.setVoltage(output);
    }
  }

  @Override
  public double getMeasurement()
  {
    return getPivotPos();
  }

  public double getPivotPos()
  {
    return encoder.getPosition();
  }

  public void pivotControl(double voltage)
  {
    motor.setVoltage(voltage);
  }

  public void zeroEncoder()
  {
    encoder.setPosition(0);
  }

  @Override
  public void periodic()
  {
    super.periodic();
    encPos.setDouble(getPivotPos());
  }
}
