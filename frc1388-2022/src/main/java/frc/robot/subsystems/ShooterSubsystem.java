// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_VictorSPX m_feederMotor;
  public enum FeederFunctions {
    FORWARD, REVERSE, OFF
  }

  private final WPI_TalonFX m_shooterMotor;

  // current rpm for shooter
  private double m_rpm = 0;
  // is the motor enabled
  private boolean m_enabled = false;

  private static final int PID_IDX = 0;

  // Math variables needed to convert RPM to ticks per second/ticks per
  private final int SENSOR_CYCLES_PER_SECOND = 10;   // sensor velocity period is 100 ms
  private final int SEC_PER_MIN = 60;
  private final int COUNTS_PER_REV = 2048;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(WPI_TalonFX shooterMotor, WPI_VictorSPX feederMotor)  {
    m_shooterMotor = shooterMotor;
    m_feederMotor = feederMotor;

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

    //Settings for feeder motor
    feederMotor.setNeutralMode(NeutralMode.Brake);

  } // end ShooterSubsystem Contructor

  public void startShooter() {
    m_enabled = true;
  }

  public boolean shooterSpeedIsReady() {
    //stub
    return true;
  
  }

  public void shooterEnabled (boolean enabled) {
    m_enabled = enabled;
  }

  public void setTargetRPM (double rpm) {
    m_rpm = rpm;
  }

  public double getTargetRPM () {
    return m_rpm;
  }

  public void setRelativeTargetRPM (double deltaRPM) {
    m_rpm = m_rpm + deltaRPM;
  }

  public void shooterRpmStepIncrease() {

  }

  public void shooterRpmStepDecrease() {

  }

  public double getRealRPM () {
    double rawSensorData = m_shooterMotor.getSelectedSensorVelocity();
    double motorRPM = rawSensorData * SENSOR_CYCLES_PER_SECOND * SEC_PER_MIN / COUNTS_PER_REV;
    return motorRPM;
  }

  public void setFeederFunction (FeederFunctions function) {
    switch (function) {
      case FORWARD:
        m_feederMotor.set(ShooterConstants.FORWARD_FEEDER_SPEED);
        break;

      case REVERSE:
        m_feederMotor.set(ShooterConstants.REVERSE_FEEDER_SPEED);
        break;

      case OFF:
        m_feederMotor.set(ShooterConstants.FEEDER_SPEED_OFF);
        break;

      default:
        m_feederMotor.set(ShooterConstants.FEEDER_SPEED_OFF);
        break;

      


    }
    

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_enabled) {
      double speed = m_rpm * COUNTS_PER_REV / SENSOR_CYCLES_PER_SECOND / SEC_PER_MIN;
      m_shooterMotor.set(ControlMode.Velocity, speed);
    } else {
      m_shooterMotor.set(0);
    }



  }
}
