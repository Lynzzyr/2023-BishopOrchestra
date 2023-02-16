// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;

public class ArmPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax m_motor1;
  private final CANSparkMax m_motor2;
  private final DutyCycleEncoder m_encoder;
  private final ShuffleboardTab sb_armTab;
  private final GenericEntry absolutePosition, angle;
  // fix the genericentry import it

  /** Creates a new ArmPIDSubsystem. */
  public ArmPIDSubsystem() {
    super(new PIDController(Constants.kArmSubsystem.kPID.kP,Constants.kArmSubsystem.kPID.kI, Constants.kArmSubsystem.kPID.kD));

    m_motor1 = new CANSparkMax(Constants.kArmSubsystem.kMotor1ID, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(Constants.kArmSubsystem.kMotor2ID, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(Constants.kArmSubsystem.kEncoderChannel);
   
    getController().setTolerance(Constants.kArmSubsystem.kPositionTolerance);
    m_motor1.restoreFactoryDefaults();
    m_motor1.setIdleMode(IdleMode.kBrake);
    m_motor1.setInverted(true);
    m_motor1.setSmartCurrentLimit(Constants.kArmSubsystem.kCurrentLimit);

    m_motor2.restoreFactoryDefaults();
    m_motor2.follow(m_motor1);
    m_motor2.setIdleMode(IdleMode.kBrake);
    m_motor2.setInverted(true);
    m_motor2.setSmartCurrentLimit(Constants.kArmSubsystem.kCurrentLimit);

    sb_armTab = Shuffleboard.getTab("Arm"); // shuffleboard tab and values
   // kP = sb_armTab.add("kP", Constants.kArmSubsystem.kPID.kP).getEntry();
   // kI = sb_armTab.add("kI", Constants.kArmSubsystem.kPID.kI).getEntry();
    // kD = sb_armTab.add("kD", Constants.kArmSubsystem.kPID.kD).getEntry();
    absolutePosition = sb_armTab.add("AbsolutePosition", 0).getEntry();
    angle = sb_armTab.add("Angle",0).getEntry();
   // rawAbsolutePosition = sb_armTab.add("RawAbsolutePosition",0).getEntry();
    setPIDFvalues(Constants.kArmSubsystem.kPID.kP, Constants.kArmSubsystem.kPID.kI, Constants.kArmSubsystem.kPID.kD);
    m_motor1.burnFlash();
    m_motor2.burnFlash();
  }

  @Override
  public void useOutput(double voltage, double setpoint) { // outputs the voltage 
    if (voltage > Constants.kArmSubsystem.kVoltageLimit - calculateFF()){
      m_motor1.setVoltage(Constants.kArmSubsystem.kVoltageLimit - calculateFF());
    }
    else if (voltage < -Constants.kArmSubsystem.kVoltageLimit - calculateFF()){
      m_motor1.setVoltage(-Constants.kArmSubsystem.kVoltageLimit - calculateFF()); 
    }
    else{
      m_motor1.setVoltage(voltage- calculateFF());
    }
    // System.out.println(voltage);
  }

  @Override
  public double getMeasurement() { // gets absolute position and returns the value 
    double ecd_value = m_encoder.getAbsolutePosition(); 

     if (ecd_value > 0.8){  // used to fix the weird values from encoder
        absolutePosition.setDouble(ecd_value - 1 + Constants.kArmSubsystem.knintydegreepos);
      return ecd_value -1 + Constants.kArmSubsystem.knintydegreepos;
    }else{
      absolutePosition.setDouble(ecd_value + Constants.kArmSubsystem.knintydegreepos);
      return ecd_value + Constants.kArmSubsystem.knintydegreepos;
    }
    // Return the process variable measurement here 
  }

  public void setPIDFvalues(double kP, double kI, double kD){ // sets PID values
    m_controller.setP(kP);
    m_controller.setI(kI);
    m_controller.setD(kD);
  }

  // public void setPIDfromshuffleboard(){ // sets PID values for shuffleboard and runs on a button
  //   setPIDFvalues(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0));
     // System.out.println("kP:" + m_controller.getP() + " kI:" + kI.getDouble(0) + " kD:" + kD.getDouble(0));
  // }

  public double calculateFF(){
    return Constants.kArmSubsystem.kg*Math.cos(Math.toRadians(getAngle()));
    
  }

  public double getAngle(){
    return getMeasurement()*360;
  }

  // public double getRawEcd(){
  //   double rawEcd_value = m_encoder.getAbsolutePosition();
  //   return rawEcd_value;
  // }

  @Override
  public void periodic() { // gets the encoder value
      super.periodic();
      getMeasurement();
      angle.setDouble(getAngle());


  }
  
}

//TO DO make it print the raw encoder setpoint, make sure the code is the same as the driveteam code, delete useless constants, pull new code from main and fix stuff 
