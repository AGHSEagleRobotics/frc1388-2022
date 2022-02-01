// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SetShooterTargetRPM;
import pabeles.concurrency.ConcurrencyOps.Reset;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_VictorSPX m_feederMotor;
  public enum FeederFunctions {
    FORWARD, REVERSE, OFF
  }

  private final WPI_TalonFX m_shooterMotor;

  // current rpm for shooter
  private double m_shooterTargetRPM = 0;
  // is the motor enabled
  private boolean m_shooterEnabled = false;
  // is the shooter at the right rpm
  private boolean m_shooterSpeedIsReady;
  private int m_shooterSpeedTest = 0;
  private static final int PID_IDX = 0;

  private Timer m_shooterCooldownTimer;

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

    m_shooterCooldownTimer = new Timer();
    m_shooterCooldownTimer.start();
  } // end constructor

  public void shooterEnabled (boolean enabled) { 
    if (enabled) {
      m_shooterEnabled = true;
    } else { // if false
      m_shooterCooldownTimer.reset(); // start cooldown timer
    }
  }

  public void setTargetRPM (double rpm) {
    m_shooterTargetRPM = rpm;
  }

  public double getTargetRPM () {
    return m_shooterTargetRPM;
  }

  public void setRelativeTargetRPM (double deltaRPM) {
    m_shooterTargetRPM = m_shooterTargetRPM + deltaRPM;
  }

  public void shooterRpmStepIncrease() { // increase
    if (m_shooterTargetRPM + ShooterConstants.SHOOTER_RPM_STEP_CHANGE <= ShooterConstants.MAX_SHOOTER_RPM) {
      m_shooterTargetRPM += ShooterConstants.SHOOTER_RPM_STEP_CHANGE;
    }
  }

  public void shooterRpmStepDecrease() { // decrease
    if (m_shooterTargetRPM - ShooterConstants.SHOOTER_RPM_STEP_CHANGE >= ShooterConstants.MIN_SHOOTER_RPM) {
      m_shooterTargetRPM -= ShooterConstants.SHOOTER_RPM_STEP_CHANGE;
    }
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
  
  public boolean shooterSpeedIsReady() {
    if (m_shooterSpeedTest >= ShooterConstants.RPM_TEST_ITERATIONS) {
      return true;
    } else {
      return false;
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_shooterCooldownTimer.hasElapsed(ShooterConstants.SHOOTER_COOLDOWN_TIME))  { // don't turn off shooter until some time has elapsed
      m_shooterEnabled = false;
    }

    if (m_shooterEnabled) {
      double speed = m_shooterTargetRPM * COUNTS_PER_REV / SENSOR_CYCLES_PER_SECOND / SEC_PER_MIN;
      m_shooterMotor.set(ControlMode.Velocity, speed);
    } else {
      m_shooterMotor.set(0);
    }

    if (Math.abs(m_shooterTargetRPM - getRealRPM()) < ShooterConstants.RPM_RANGE) {
        m_shooterSpeedTest++;
    } else {
      m_shooterSpeedTest = 0;
    }

  }  // END periodic()
  

} // END class ShooterSubsystem
