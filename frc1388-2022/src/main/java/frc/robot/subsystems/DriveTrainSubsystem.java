// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  //define class fields
  private final DifferentialDrive m_differentialDrive;

  private boolean m_isReverse = false;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(WPI_TalonFX leftFront, WPI_TalonFX leftBack, WPI_TalonFX rightFront, WPI_TalonFX rightBack) {

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    //Inverting left motor
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
    m_differentialDrive.setDeadband(DriveTrainConstants.DEADBAND); 

    //set all motors to break
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    
    //add to shuffle board
    addChild("DifferentialDrive", m_differentialDrive);

  }
  
  //constructor for working on Knightmare!!
  // public DriveTrainSubsystem(WPI_VictorSPX leftFront, WPI_TalonSRX leftBack, WPI_TalonSRX rightFront, WPI_VictorSPX rightBack) {

  //   leftBack.follow(leftFront);
  //   rightBack.follow(rightFront);

  //   //Needed to invert left side
  //   leftFront.setInverted(false);
  //   leftBack.setInverted(false);

  //   m_differentialDrive = new DifferentialDrive(leftFront, rightFront);

  //   //set differentialDrive parameters
  //   m_differentialDrive.setSafetyEnabled(true);
  //   m_differentialDrive.setExpiration(DriveTrainConstants.EXPIRATION);
  //   m_differentialDrive.setMaxOutput(DriveTrainConstants.MAX_OUTPUT);
  //   m_differentialDrive.setDeadband(DriveTrainConstants.DEADBAND); 

  //   //set all motors to break
  //   leftFront.setNeutralMode(NeutralMode.Brake);
  //   leftBack.setNeutralMode(NeutralMode.Brake);
  //   rightFront.setNeutralMode(NeutralMode.Brake);
  //   rightBack.setNeutralMode(NeutralMode.Brake);
    
  //   //add to shuffle board
  //   addChild("DifferentialDrive", m_differentialDrive);

  // }



  public void arcadeDrive (double xSpeed, double zRotation) {
    if (!m_isReverse) {
      m_differentialDrive.arcadeDrive(xSpeed, zRotation);
    } else {
      m_differentialDrive.arcadeDrive(-xSpeed, zRotation);
    }
  }

  public void curvatureDrive (double xSpeed, double zRotation, boolean isQuickTurn) {
    if (!m_isReverse) {
      m_differentialDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    } else {
      m_differentialDrive.curvatureDrive(-xSpeed, zRotation, isQuickTurn);
    }
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    if (!m_isReverse) {
      m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
    } else {
      m_differentialDrive.tankDrive(-rightSpeed, -leftSpeed);
    }
  }

  public void toggleReverse() {
    m_isReverse = !m_isReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

