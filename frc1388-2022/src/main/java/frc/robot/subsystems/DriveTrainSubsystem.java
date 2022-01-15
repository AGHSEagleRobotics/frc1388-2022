// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  //define class fields
  private final DifferentialDrive m_differentialDrive;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(WPI_TalonFX leftFront, WPI_TalonFX leftBack, WPI_TalonFX rightFront, WPI_TalonFX rightBack) {

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    m_differentialDrive = new DifferentialDrive(leftFront, rightFront);

    //set diferentialDrive parameters
    m_differentialDrive.setSafetyEnabled(true);
    m_differentialDrive.setExpiration(DriveTrainConstants.EXPIRATION);
    m_differentialDrive.setMaxOutput(DriveTrainConstants.MAX_OUTPUT);
    m_differentialDrive.setDeadband(DriveTrainConstants.DEADBAND); 

    //set all moters to break
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    
    //add to shuffle board
    addChild("DifferentialDrive", m_differentialDrive);

  }



  public void arcadeDrive (double xSpeed, double zRotation) {
    m_differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void curvatureDrive (double xSpeed, double zRotation, boolean isQuickTurn) {
    m_differentialDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

