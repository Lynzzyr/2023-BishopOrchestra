// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.kLimelight.KAutoDriveAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AutoDriveAlign extends CommandBase {
  private final Limelight sys_Limelight;
  private final Drivetrain sys_Drivetrain;
  private final PIDController m_pidController;
  private CommandXboxController m_joystick;
  private boolean finished = false; 
  private double calculatedOutput, finishTimer;

  public AutoDriveAlign(Limelight limelight, Drivetrain drivetrain) {
    //initializing
    sys_Limelight = limelight;
    sys_Drivetrain = drivetrain;
    m_joystick = null; 

    //PID controller
    m_pidController = new PIDController(KAutoDriveAlign.kP, KAutoDriveAlign.kI, KAutoDriveAlign.kD);
    m_pidController.setSetpoint(KAutoDriveAlign.desiredTargetDistance);
    m_pidController.setTolerance(KAutoDriveAlign.driveTolerance);
  }

  public AutoDriveAlign(Limelight limelight, Drivetrain drivetrain, CommandXboxController joystick) {
    //initializes default constructor
    this(limelight, drivetrain);
    m_joystick = joystick;
  }

  /**
   * Calculates the optimal robot rotation speed given target position through PID
   * @return Target speed
   */
  public double getTargetSpeed(){
    double targetDistance = sys_Limelight.getTargetDistance();
    calculatedOutput = m_pidController.calculate(targetDistance);

    //applying feat foward and secondary tolerance
    if (calculatedOutput >= 0){
      calculatedOutput += KAutoDriveAlign.driveFF;
      if (KAutoDriveAlign.applySecondaryTolerance && targetDistance <= KAutoDriveAlign.driveTolerance){     //applying secondary tolerance
        calculatedOutput = 0; 
      }
    } else if (calculatedOutput < 0){
      calculatedOutput -= KAutoDriveAlign.driveFF;
      if (KAutoDriveAlign.applySecondaryTolerance && targetDistance >= -KAutoDriveAlign.driveTolerance){    //applying secondary tolerance
        calculatedOutput = 0;  
      }
    }

    return calculatedOutput;
  }

  /**
   * Retrieves the current joystick rotation input.
   * @return Left joystick Y-input.
   */
  public double getZSpeed() {
    double zSpeed = 0; 
    if (m_joystick != null){
      zSpeed = m_joystick.getLeftY(); //check if right
    }

    return zSpeed; 
  }

  /**
   * Triggers endstate if the target has been reached for a period of time. 
   */
  public void alignTimeout(){
    if (getTargetSpeed() != 0){
      finishTimer = System.currentTimeMillis();
    } else if ((System.currentTimeMillis() - finishTimer) >= KAutoDriveAlign.alignedTimeout){
      finished = true; 
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_Limelight.turnOn();
    sys_Limelight.setData("pipeline", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sys_Drivetrain.autoTurnDrive(getTargetSpeed(), getZSpeed());
    alignTimeout();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_Limelight.turnOff();
    sys_Drivetrain.autoTurnDrive(0, 0);
    if (KAutoDriveAlign.debugMode){
      System.out.println("[AutoDriveAlign] Command finsihed");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
