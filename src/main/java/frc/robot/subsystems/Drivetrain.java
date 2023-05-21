package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kDriveteam;
import frc.robot.Constants.kDrivetrain.kMotor;
import frc.robot.Constants.kGyro;

public class Drivetrain extends SubsystemBase {

    private final WPI_TalonFX mot_leftFrontDrive;
    private final WPI_TalonFX mot_leftCentreDrive;
    private final WPI_TalonFX mot_leftRearDrive;

    private final WPI_TalonFX mot_rightFrontDrive;
    private final WPI_TalonFX mot_rightCentreDrive;
    private final WPI_TalonFX mot_rightRearDrive;

    private final SupplyCurrentLimitConfiguration m_currentLimit;

    private NeutralMode m_neutralMode;

    private final DifferentialDrive m_diffDrive;

    private final ArrayList<TalonFX> m_instruments;
    private final Orchestra m_orchestra;

    private final WPI_CANCoder enc_leftDrive;
    private final WPI_CANCoder enc_rightDrive;
    private final CANCoderConfiguration enc_config;

    private final WPI_Pigeon2 m_gyro;
    private final DifferentialDriveOdometry m_odometry;

    private double currentRampRate = kDriveteam.rampRate;
    private double forwardSpeed = kDriveteam.defaultSpeedMultiplier;
    private double turningSpeed = kDriveteam.defaultTurningMultiplier;

    boolean debugMode = false; // Show Shuffleboard items

    private ShuffleboardTab sb_drivetrainTab;
    private GenericEntry nt_leftVelocity;
    private GenericEntry nt_rightVelocity;
    private GenericEntry nt_leftDistance;
    private GenericEntry nt_rightDistance;
    private GenericEntry nt_leftTemperature;
    private GenericEntry nt_rightTemperature;
    private GenericEntry nt_gyroYaw;
    private GenericEntry nt_gyroPitch;
    private GenericEntry nt_gyroRoll;
    private GenericEntry nt_poseMetersX;
    private GenericEntry nt_poseMetersY;


    public Drivetrain() {

        // Instantiate motors and differential drive
        mot_leftFrontDrive = new WPI_TalonFX(kDrivetrain.kMotor.id_leftFrontDrive);
        mot_leftCentreDrive = new WPI_TalonFX(kDrivetrain.kMotor.id_leftCentreDrive);
        mot_leftRearDrive = new WPI_TalonFX(kDrivetrain.kMotor.id_leftRearDrive);

        mot_rightFrontDrive = new WPI_TalonFX(kDrivetrain.kMotor.id_rightFrontDrive);
        mot_rightCentreDrive = new WPI_TalonFX(kDrivetrain.kMotor.id_rightCentreDrive);
        mot_rightRearDrive = new WPI_TalonFX(kDrivetrain.kMotor.id_rightRearDrive);

        // Current limiting
        m_currentLimit = new SupplyCurrentLimitConfiguration();
        m_currentLimit.enable = true;
        m_currentLimit.currentLimit = kMotor.currentLimit;

        m_neutralMode = NeutralMode.Brake;

        configMotors();

        m_diffDrive = new DifferentialDrive(mot_leftFrontDrive, mot_rightFrontDrive);

        m_instruments = new ArrayList<TalonFX>();
        m_instruments.add(mot_leftFrontDrive);
        m_instruments.add(mot_leftCentreDrive);
        m_instruments.add(mot_leftRearDrive);
        m_instruments.add(mot_rightCentreDrive);

        m_orchestra = new Orchestra(m_instruments);

        // Instantiate CANCoders
        enc_leftDrive = new WPI_CANCoder(kDrivetrain.kCANCoder.id_leftEncoder);
        enc_rightDrive = new WPI_CANCoder(kDrivetrain.kCANCoder.id_rightEncoder);

        enc_config = new CANCoderConfiguration();
        enc_config.sensorCoefficient = kDrivetrain.kCANCoder.enc_SensorCoefficient;
        enc_config.unitString = kDrivetrain.kCANCoder.enc_UnitString;
        enc_config.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_leftDrive.configAllSettings(enc_config);
        enc_rightDrive.configAllSettings(enc_config);

        resetEncoders();

        // Gyro and odometry
        m_gyro = new WPI_Pigeon2(kGyro.id_gyro);
        m_gyro.configMountPose(kGyro.mountPoseForward, kGyro.mountPoseUp);

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());

        resetGyro();

        // Shuffleboard
        if (debugMode) {
            sb_drivetrainTab = Shuffleboard.getTab("Drivetrain");
            nt_leftVelocity = sb_drivetrainTab.add("Left velocity", getLeftVelocity()).getEntry();
            nt_rightVelocity = sb_drivetrainTab.add("Right velocity", getRightVelocity()).getEntry();
            nt_leftDistance = sb_drivetrainTab.add("Left distance", getLeftDistance()).getEntry();
            nt_rightDistance = sb_drivetrainTab.add("Right distance", getRightDistance()).getEntry();
            nt_leftTemperature = sb_drivetrainTab.add("Left temperature", getAverageLeftMotorTemperature()).getEntry();
            nt_rightTemperature = sb_drivetrainTab.add("Right temperature", getAverageRightMotorTemperature()).getEntry();
            nt_gyroYaw = sb_drivetrainTab.add("Gyro yaw", getYaw()).getEntry();
            nt_gyroPitch = sb_drivetrainTab.add("Gyro pitch", getPitch()).getEntry();
            nt_gyroRoll = sb_drivetrainTab.add("Gyro roll", getRoll()).getEntry();
            nt_poseMetersX = sb_drivetrainTab.add("X Pose meters", m_odometry.getPoseMeters().getX()).getEntry();
            nt_poseMetersY = sb_drivetrainTab.add("Y Pose meters", m_odometry.getPoseMeters().getY()).getEntry();
        }
    }

    /**
     * Config factory default
     * 
     * Set followers
     * 
     * Invert left side
     * 
     * Current limiting
     * 
     * Ramp rate
     * 
     * Brake mode
     */
    private void configMotors() {
        // Reset factory default
        mot_leftFrontDrive.configFactoryDefault();
        mot_leftCentreDrive.configFactoryDefault();
        mot_leftRearDrive.configFactoryDefault();

        mot_rightFrontDrive.configFactoryDefault();
        mot_rightCentreDrive.configFactoryDefault();
        mot_rightRearDrive.configFactoryDefault();

        // Set followers
        mot_leftCentreDrive.follow(mot_leftFrontDrive);
        mot_leftRearDrive.follow(mot_leftFrontDrive);

        mot_rightCentreDrive.follow(mot_rightFrontDrive);
        mot_rightRearDrive.follow(mot_rightFrontDrive);

        // Invert left side
        mot_leftFrontDrive.setInverted(true);
        mot_leftCentreDrive.setInverted(true);
        mot_leftRearDrive.setInverted(true);

        // Current limiting
        mot_leftFrontDrive.configSupplyCurrentLimit(m_currentLimit);
        mot_leftCentreDrive.configSupplyCurrentLimit(m_currentLimit);
        mot_leftRearDrive.configSupplyCurrentLimit(m_currentLimit);

        mot_rightFrontDrive.configSupplyCurrentLimit(m_currentLimit);
        mot_rightCentreDrive.configSupplyCurrentLimit(m_currentLimit);
        mot_rightRearDrive.configSupplyCurrentLimit(m_currentLimit);

        // Set brake mode
        setNeutralMode(m_neutralMode);

        // Ramp rate
        // rampRate(kDrivetrain.kDriveteam.rampRate);
    }

    /**
     * Set ramp rate on motors
     * @param seconds time to get to desired speed
     */
    public void rampRate(double seconds) {
        //sets the ramprate to all the motors
        mot_leftFrontDrive.configOpenloopRamp(seconds);
        mot_leftCentreDrive.configOpenloopRamp(seconds);
        mot_leftRearDrive.configOpenloopRamp(seconds);

        mot_rightFrontDrive.configOpenloopRamp(seconds);
        mot_rightCentreDrive.configOpenloopRamp(seconds);
        mot_rightRearDrive.configOpenloopRamp(seconds);

        currentRampRate = seconds;
    }

    /**
     * Gets the current ramprate
     * @return seconds
     */

    public double getRampRate() {
        return currentRampRate;
    }

    /**
     * Arcade drive
     * @param xSpeed forward speed
     * @param zRotation rotation
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        m_diffDrive.arcadeDrive(
            xSpeed * forwardSpeed,
            zRotation * turningSpeed);
    }

    public void autoTurnDrive(double xSpeed, double zRotation) {
        m_diffDrive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Tank drive voltages, for trajectory
     * @param leftVolts
     * @param rightVolts
     */
    public void tankDriveVoltages(double leftVolts, double rightVolts) {
        mot_leftFrontDrive.setVoltage(leftVolts);
        mot_rightFrontDrive.setVoltage(rightVolts);
        m_diffDrive.feed();
    }

    /**
     * Tank Drive % speeds, for setting motor speeds
     * @param speed speed at which the drivetrain should spin at
     */

    public void tankDriveSpeeds(double speed) {
        mot_leftFrontDrive.set(speed);
        mot_rightFrontDrive.set(speed);
        m_diffDrive.feed();
    }

    /**
     * Set brake/coast mode
     * @param newMode
     */
    public void setNeutralMode(NeutralMode newMode) {
        m_neutralMode = newMode;
        mot_leftFrontDrive.setNeutralMode(newMode);
        mot_leftCentreDrive.setNeutralMode(newMode);
        mot_leftRearDrive.setNeutralMode(newMode);
        mot_rightFrontDrive.setNeutralMode(newMode);
        mot_rightCentreDrive.setNeutralMode(newMode);
        mot_rightRearDrive.setNeutralMode(newMode);
    }

    /**
     * Toggles neutral mode
     */

    public void toggleNeutralMode() {
        if (getNeutralMode() == NeutralMode.Brake) {
            setNeutralMode(NeutralMode.Coast);
        } else {
            setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * Get brake/coast mode
     * @return brake / coast mode
     */
    public NeutralMode getNeutralMode() {
        return m_neutralMode;
    }

    public double getAverageLeftMotorTemperature() {
        return (mot_leftFrontDrive.getTemperature() + mot_leftCentreDrive.getTemperature() + mot_leftRearDrive.getTemperature()) / 3;
    }

    public double getAverageRightMotorTemperature() {
        return (mot_rightFrontDrive.getTemperature() + mot_rightCentreDrive.getTemperature() + mot_rightRearDrive.getTemperature()) / 3;
    }

    // CANCoders ----------

    /**
     * Set encoder position to 0
     */
    public void resetEncoders() {
        enc_leftDrive.setPosition(0);
        enc_rightDrive.setPosition(0);
    }

    /**
     * @return left encoder distance in metres
     */
    public double getLeftDistance() {
        return -enc_leftDrive.getPosition();
    }

    /**
     * @return right encoder distance in metres
     */
    public double getRightDistance() {
        return enc_rightDrive.getPosition();
    }

    /**
     * @return left encoder velocity in metres per second
     */
    public double getLeftVelocity() {
        return -enc_leftDrive.getVelocity();
    }

    /**
     * @return right encoder velocity in metres per second
     */
    public double getRightVelocity() {
        return enc_rightDrive.getVelocity();
    }

    /**
     * Get the absolute motor velocitys
     * @return The absolute motor speeds
     */

    public double getAverageVelocity() {
        return (Math.abs(getLeftVelocity()) + Math.abs(getRightVelocity())) / 2;
    }

    // ----------

    // Gyro and odometry

    /**
     * Reset gyro heading to zero
     */
    public void resetGyro() {
        m_gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }
    
    public double getHeading() {
        return getRotation2d().getDegrees();
    }
    
    public double getYaw() {
        return m_gyro.getYaw();
    }
    
    public double getPitch() {
        return m_gyro.getPitch();
    }
    
    public double getRoll() {
        return m_gyro.getRoll();
    }


    public double getTurnRate() {
        return m_gyro.getRate();
    }

    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    /**
     * Reset the odometry to a specified pose.
     * 
     * The reset position command has 0, 0 (distance) hard-coded in to
     * avoid any encoder timing lag from resetting encoders.
     * 
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
    }

    /**
     * Sets the speed multiplier for the network table entry
     * @param speed x speed
     * @param turningSpeed z rotation
     */

    public void setSpeed(double speed, double turningSpeed) {
        //updating the multipliers for the drive
        forwardSpeed = speed;
        this.turningSpeed = turningSpeed;
    }

    // Orchestra ----------

    public void setTrack(String filePath) {

        m_orchestra.loadMusic(filePath);

    }

    /**
     * Toggles music to play or pause.
     */
    public void toggleMusic() {

        if (isPlayingMusic()) {

            m_orchestra.pause();

        } else {

            m_orchestra.play();

        }

    }

    /**
     * Stops the music file that is loaded.
     * This resets the current position in the track to the start.
     */

    public void stopAndRewindMusic() {

        m_orchestra.stop();

    }

    public boolean isPlayingMusic() {

        return m_orchestra.isPlaying();

    }

    @Override
    public void periodic() {
        // Update odometry
        m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());

        // Push data to Shuffleboard
        if (debugMode) {
            nt_leftVelocity.setDouble(getLeftVelocity());
            nt_rightVelocity.setDouble(getRightVelocity());
            nt_leftDistance.setDouble(getLeftDistance());
            nt_rightDistance.setDouble(getRightDistance());
            nt_leftTemperature.setDouble(getAverageLeftMotorTemperature());
            nt_rightTemperature.setDouble(getAverageRightMotorTemperature());
            nt_gyroYaw.setDouble(getYaw());
            nt_gyroPitch.setDouble(getPitch());
            nt_gyroRoll.setDouble(getRoll());
            nt_poseMetersY.setDouble(m_odometry.getPoseMeters().getY());
            nt_poseMetersX.setDouble(m_odometry.getPoseMeters().getX());
        }
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}