// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_shooterMotor;

  private static final int PID_IDX = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(WPI_TalonFX shooterMotor) {
    m_shooterMotor = shooterMotor;
    
    // Factory Default all hardware to prevent unexpected behaviour
    m_shooterMotor.configFactoryDefault();
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_shooterMotor.setSensorPhase(false);
    m_shooterMotor.setInverted(false);

    m_shooterMotor.config_kF(PID_IDX, ShooterConstants.GAINS_VELOCITY_F);
    m_shooterMotor.config_kP(PID_IDX, ShooterConstants.GAINS_VELOCITY_P);
    m_shooterMotor.config_kI(PID_IDX, ShooterConstants.GAINS_VELOCITY_I);
    m_shooterMotor.config_kD(PID_IDX, ShooterConstants.GAINS_VELOCITY_D);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
