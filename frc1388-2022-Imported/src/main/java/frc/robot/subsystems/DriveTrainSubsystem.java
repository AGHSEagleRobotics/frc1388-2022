// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class DriveTrainSubsystem extends SubsystemBase {

  private static final Logger log = LogManager.getLogger(DriveTrainSubsystem.class);


  //define class fields
  private final DifferentialDrive m_differentialDrive;
  private final ADIS16470_IMU m_gyro;

  private boolean m_isReverse = false;

  private final WPI_TalonFX m_leftFront;
  private final WPI_TalonFX m_leftBack;
  private final WPI_TalonFX m_rightFront;
  private final WPI_TalonFX m_rightBack;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(WPI_TalonFX leftFront, WPI_TalonFX leftBack, WPI_TalonFX rightFront, WPI_TalonFX rightBack, ADIS16470_IMU gyro) {

    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    //Invert left motor
    leftFront.setInverted(true);
    leftBack.setInverted(true);
    //Don't invert right motor
    rightFront.setInverted(false);
    rightBack.setInverted(false);

    //Differential drive
    m_differentialDrive = new DifferentialDrive(leftFront, rightFront);

    //set differentialDrive parameters
    m_differentialDrive.setSafetyEnabled(true);
    m_differentialDrive.setExpiration(DriveTrainConstants.EXPIRATION);
    m_differentialDrive.setMaxOutput(DriveTrainConstants.MAX_OUTPUT);

    //set all motors to brake
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    m_gyro = gyro;
    m_gyro.calibrate();
    m_gyro.reset();
    m_gyro.setYawAxis(IMUAxis.kZ);
    
    //add to shuffle board
    addChild("DifferentialDrive", m_differentialDrive);
    addChild("Gyro", m_gyro);

  }

  public void resetLeftEncoder(){
    m_leftFront.setSelectedSensorPosition(0);
  }

  public void resetRightEncoder(){
    m_rightFront.setSelectedSensorPosition(0);
  }

  //Not in use
  public void arcadeDrive (double xSpeed, double zRotation) {
    if (!m_isReverse) {
      m_differentialDrive.arcadeDrive(xSpeed, zRotation);
    } else {
      m_differentialDrive.arcadeDrive(-xSpeed, zRotation);
    }
  }

  public void curvatureDrive (double xSpeed, double zRotation, boolean precisionMode) {
    if (!m_isReverse) {
      m_differentialDrive.curvatureDrive(xSpeed, zRotation, precisionMode);
    } else {
      m_differentialDrive.curvatureDrive(-xSpeed, zRotation, precisionMode);
    }
  }

  //Not in use
  public void tankDrive(double leftSpeed, double rightSpeed){
    if (!m_isReverse) {
      m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
    } else {
      m_differentialDrive.tankDrive(-rightSpeed, -leftSpeed);
    }
  }

  public void setForward(boolean isForewards) {
    m_isReverse = !isForewards;
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getGyroAngle() {
    return -(m_gyro.getAngle());
  }

  public double getRightEncoderDistance(){
    return m_rightFront.getSelectedSensorPosition() 
    * DriveTrainConstants.INCHES_PER_ENCODER_UNITS 
    * DriveTrainConstants.WHEEL_CIRCUMFERENCE;
  }
  public double getLeftEncoderDistance(){
    return m_leftFront.getSelectedSensorPosition() 
    * DriveTrainConstants.INCHES_PER_ENCODER_UNITS;
  }
  
  public void setNeutralMode(NeutralMode mode) {
    m_leftFront.setNeutralMode(mode);
    m_leftBack.setNeutralMode(mode);
    m_rightFront.setNeutralMode(mode);
    m_rightBack.setNeutralMode(mode);

  }

  public void setDeadbandZero() {
    m_differentialDrive.setDeadband(0); 
  }

  public void setDeadbandDefault() {
    m_differentialDrive.setDeadband(DriveTrainConstants.DEADBAND);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // log.info("Rotation: {}, X angle: {}, Y angle: {}", m_gyro.getAngle(), m_gyro.getXComplementaryAngle(), m_gyro.getYComplementaryAngle());
    // log.debug("Y angle: {}", m_gyro.getAngle());
  }
}

