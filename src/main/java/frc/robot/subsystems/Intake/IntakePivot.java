// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.kIntake;
import frc.robot.Constants.kIntake.kVoltageLimits;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class IntakePivot extends PIDSubsystem
{
  private final CANSparkMax motor;
  private final DutyCycleEncoder encoder;

  private final ShuffleboardTab tab_intake;
  private final GenericEntry kP, kI, kD, encPos;

  public IntakePivot()
  {
    super(new PIDController(kIntake.kPivotP, kIntake.kPivotI, kIntake.kPivotD));

    motor = new CANSparkMax(kIntake.id_motPivot, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(kIntake.kIntakeCurrentLimit);
    motor.setInverted(true);
    motor.burnFlash();

    encoder = new DutyCycleEncoder(kIntake.port_encPivot);

    tab_intake = Shuffleboard.getTab("Intake");
    kP = tab_intake.add("kPivotP", kIntake.kPivotP).getEntry();
    kI = tab_intake.add("kPivotI", kIntake.kPivotI).getEntry();
    kD = tab_intake.add("kPivotD", kIntake.kPivotD).getEntry();
    encPos = tab_intake.add("Pivot Abs Pos", getMeasurement()).getEntry();
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
    return encoder.getAbsolutePosition();
  }

  public void pivotControl(double voltage)
  {
    motor.setVoltage(voltage);
  }

  @Override
  public void periodic()
  {
    super.periodic();
    encPos.setDouble(encoder.getAbsolutePosition());
  }
}
