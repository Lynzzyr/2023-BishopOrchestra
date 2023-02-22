// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.kIntake;

// public class Intake_old extends SubsystemBase
// {
//   private final CANSparkMax pivot;
//   private final CANSparkMax wrist;
//   private final WPI_TalonFX roller;

//   private final SparkMaxPIDController pid_pivot;
//   private final SparkMaxPIDController pid_wrist;

//   private final SparkMaxAbsoluteEncoder enc_pivot;
//   private final SparkMaxAbsoluteEncoder enc_wrist;

//   private final ShuffleboardTab tab_intake;
//   private final GenericEntry pos_encPivot, pos_encWrist;
  
//   public Intake_old()
//   {
//     pivot = new CANSparkMax(kIntake.id_motPivot, MotorType.kBrushless);
//     pivot.restoreFactoryDefaults();
//     pivot.setIdleMode(IdleMode.kBrake);

//     wrist = new CANSparkMax(kIntake.id_motPivot, MotorType.kBrushless);
//     wrist.restoreFactoryDefaults();
//     wrist.setIdleMode(IdleMode.kBrake);

//     roller = new WPI_TalonFX(kIntake.id_motRoller);
//     roller.configFactoryDefault();
//     roller.setNeutralMode(NeutralMode.Brake);

//     enc_pivot = pivot.getAbsoluteEncoder(Type.kDutyCycle);
//     enc_wrist = wrist.getAbsoluteEncoder(Type.kDutyCycle);

//     pid_pivot = pivot.getPIDController();
//     setPivotPID();
    
//     pid_wrist = wrist.getPIDController();
//     setWristPID();
    
//     pivot.burnFlash();
//     wrist.burnFlash();

//     tab_intake = Shuffleboard.getTab("Intake");
//     pos_encPivot = tab_intake.add("Pivot Pos", getPivotPos()).getEntry();
//     pos_encWrist = tab_intake.add("Wrist Pos", getWristPos()).getEntry();
//   }

//   public void setPivotPID() {
//     pid_pivot.setFeedbackDevice(enc_pivot);

//     pid_pivot.setP(kIntake.kPivotP);
//     pid_pivot.setI(kIntake.kPivotI);
//     pid_pivot.setD(kIntake.kPivotD);

//     pid_pivot.setPositionPIDWrappingEnabled(true);
//     pid_pivot.setPositionPIDWrappingMinInput(0);
//     pid_pivot.setPositionPIDWrappingMaxInput(1);
//   }

//   public void setWristPID() {
//     pid_wrist.setP(kIntake.kWristP);
//     pid_wrist.setI(kIntake.kWristI);
//     pid_wrist.setD(kIntake.kWristD);

//     pid_wrist.setPositionPIDWrappingEnabled(true);
//     pid_wrist.setPositionPIDWrappingMinInput(0);
//     pid_wrist.setPositionPIDWrappingMaxInput(1);
//   }

//   public double getPivotPos()
//   {
//     return enc_pivot.getPosition();
//   }

//   public double getWristPos()
//   {
//     return enc_wrist.getPosition();
//   }

//   public void pivotControl(double voltage)
//   {
//     pivot.setVoltage(voltage);
//   }

//   public void setPivotDirection(boolean inverted)
//   {
//     if (inverted)
//     {
//       pivot.setInverted(true);
//     }
//     else
//     {
//       pivot.setInverted(false);
//     }
//   }

//   public void wristControl(double voltage)
//   {
//     wrist.setVoltage(voltage);
//   }

//   public void setWristDirection(boolean inverted)
//   {
//     if (inverted)
//     {
//       wrist.setInverted(true);
//     }
//     else
//     {
//       wrist.setInverted(false);
//     }
//   }

//   public void rollerControl(double voltage)
//   {
//     roller.setVoltage(voltage);
//   }

//   public void setRollerDirection(boolean inverted)
//   {
//     if (inverted)
//     {
//       roller.setInverted(true);
//     }
//     else
//     {
//       roller.setInverted(false);
//     }
//   }

//   public void pivotToSetpoint(double setpoint) {
//     pid_pivot.setReference(setpoint, ControlType.kPosition);
//   }

//   public void wristToSetpoint(double setpoint) {
//     pid_wrist.setReference(setpoint, ControlType.kPosition);
//   }

//   @Override
//   public void periodic()
//   {
//     pos_encPivot.setDouble(getPivotPos());
//     pos_encWrist.setDouble(getWristPos());
//   }
// }
