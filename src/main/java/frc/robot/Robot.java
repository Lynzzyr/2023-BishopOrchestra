// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kTrajectoryPath;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.commands.SetCoastMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Path following trajectory
  private PathPlannerTrajectory trajectory;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Load trajectory paths
    trajectory = PathPlanner.loadPath(kTrajectoryPath.path1, new PathConstraints(kAuto.kMaxSpeed, kAuto.kMaxAcceleration));

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(trajectory);

    // Set coast mode after 5 seconds disabled
    new Trigger(this::isEnabled)
      .negate()
      .debounce(5)
      .onTrue(new SetCoastMode(m_robotContainer.sys_drivetrain));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (m_robotContainer.sys_candle.getCurrentAnimation() != 4) {
      // m_robotContainer.sys_candle.idleAnimation();
      Commands.runOnce(m_robotContainer.sys_candle::idleAnimation).ignoringDisable(true).schedule();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_robotContainer.sys_candle.inGameAnimation();
    Commands.runOnce(m_robotContainer.sys_candle::inGameAnimation).ignoringDisable(true).schedule();

    // Set brake mode
    m_robotContainer.sys_drivetrain.setNeutralMode(NeutralMode.Brake);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (DriverStation.getMatchTime() <= 0.1) {
      // m_robotContainer.sys_candle.chargedUp();
      Commands.runOnce(m_robotContainer.sys_candle::chargedUp).ignoringDisable(true).schedule();
    }
  }

  @Override
  public void teleopInit() {
    //TODO: Remove this later
    m_robotContainer.sys_claw.zeroEncoder();
    // Set in game animation
    // m_robotContainer.sys_candle.inGameAnimation();
    Commands.runOnce(m_robotContainer.sys_candle::inGameAnimation).ignoringDisable(true).schedule();

    // Set brake mode
    m_robotContainer.sys_drivetrain.setNeutralMode(NeutralMode.Brake);
    //Commands.runOnce((() -> new ArmRotation(m_robotContainer.sys_ArmPIDSubsystem, Constants.kArmSubsystem.Setpoints.kdrivingpos))).schedule();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {

    // Set brake mode
    m_robotContainer.sys_drivetrain.setNeutralMode(NeutralMode.Brake);

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}