// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Arm extends PIDSubsystem {
  // motors and encoder
  private final CANSparkMax m_motor1;
  private final CANSparkMax m_motor2;
  private final DutyCycleEncoder m_encoder;

  // shufflebaord 
  private  ShuffleboardTab sb_armTab;
  private  GenericEntry absolutePosition, angle, rawPosition;

  private double prevPos;

  // debug mode
  private final boolean debug = false;

  /** Creates a new Arm. */
  public Arm() {
    super(new PIDController(Constants.kArmSubsystem.kPID.kP,Constants.kArmSubsystem.kPID.kI, Constants.kArmSubsystem.kPID.kD));

    // Instantiate motors and encoder
    m_motor1 = new CANSparkMax(Constants.kArmSubsystem.kMotor1ID, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(Constants.kArmSubsystem.kMotor2ID, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(Constants.kArmSubsystem.kEncoderChannel);
   
    getController().setTolerance(Constants.kArmSubsystem.kPositionTolerance);

    // configurate motor 1
    m_motor1.restoreFactoryDefaults();
    m_motor1.setIdleMode(IdleMode.kBrake);
    m_motor1.setInverted(true);
    m_motor1.setSmartCurrentLimit(Constants.kArmSubsystem.kCurrentLimit);

    // congifurate motor 2
    m_motor2.restoreFactoryDefaults();
    m_motor2.follow(m_motor1);
    m_motor2.setIdleMode(IdleMode.kBrake);
    m_motor2.setInverted(true);
    m_motor2.setSmartCurrentLimit(Constants.kArmSubsystem.kCurrentLimit);

    // debug mode, shuffleboard tab and values
    if (debug){
      sb_armTab = Shuffleboard.getTab("Arm"); 
      absolutePosition = sb_armTab.add("AbsolutePosition", 0).getEntry();
      angle = sb_armTab.add("Angle",0).getEntry();
      rawPosition = sb_armTab.add("rawPosition",0).getEntry();

      //kP = sb_armTab.add("kP", Constants.kArmSubsystem.kPID.kP).getEntry();
      //kI = sb_armTab.add("kI", Constants.kArmSubsystem.kPID.kI).getEntry();
      //kD = sb_armTab.add("kD", Constants.kArmSubsystem.kPID.kD).getEntry();

    }
    setPIDFvalues(Constants.kArmSubsystem.kPID.kP, Constants.kArmSubsystem.kPID.kI, Constants.kArmSubsystem.kPID.kD);
    m_motor1.burnFlash();
    m_motor2.burnFlash();
  }

  @Override
  public void useOutput(double voltage, double setpoint) { 
    // outputs the voltage with limit and feedforward
    if (voltage > Constants.kArmSubsystem.kVoltageLimit - calculateFF()){
      m_motor1.setVoltage(Constants.kArmSubsystem.kVoltageLimit - calculateFF());
    }
    else if (voltage < -Constants.kArmSubsystem.kVoltageLimit - calculateFF()){
      m_motor1.setVoltage(-Constants.kArmSubsystem.kVoltageLimit - calculateFF()); 
    }
    else{
      m_motor1.setVoltage(voltage- calculateFF());
    }
  }

  @Override
  public double getMeasurement() { 
    // gets absolute position and returns the value
    double ecd_value = m_encoder.getAbsolutePosition(); 

    // debug
    if (debug)
      rawPosition.setDouble(ecd_value);
    
    // used to fix encoder values so the values are consistent 
    // 0.4 indicates the encoder value the shoulder will never reach
    if (ecd_value < 0.4){  
      if (debug){
        absolutePosition.setDouble(ecd_value + 1 );
      }
      return ecd_value +1;

    }else{
      if (debug){
        absolutePosition.setDouble(ecd_value); 
      }
      return ecd_value;
    }
  }

  // sets the PID values
  public void setPIDFvalues(double kP, double kI, double kD){ 
    m_controller.setP(kP);
    m_controller.setI(kI);
    m_controller.setD(kD);
  }

  // sets PID values for shuffleboard that runs on a button
  // public void setPIDfromshuffleboard(){ 
  //   setPIDFvalues(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0));
     // System.out.println("kP:" + m_controller.getP() + " kI:" + kI.getDouble(0) + " kD:" + kD.getDouble(0));
  // }

  // calculates feedforward to use for voltage
  public double calculateFF(){
    return Constants.kArmSubsystem.kg*Math.cos(Math.toRadians(getAngle()));
  }

  // angle of the arm 
  public double getAngle(){
    return (getMeasurement()*360) - 397;
  }

  public double getPrevPos() {
    return prevPos;
  }

  public void setPrevPos(double prevPos) {
    this.prevPos = prevPos;
  }

  // sets voltage for manual control
  public void moveVolts(double volts) {
    m_motor1.setVoltage(volts);
  }

  @Override
  public void periodic() { 
      super.periodic();
      getMeasurement();
      if (debug){
      angle.setDouble(getAngle());
      }
  }  
}