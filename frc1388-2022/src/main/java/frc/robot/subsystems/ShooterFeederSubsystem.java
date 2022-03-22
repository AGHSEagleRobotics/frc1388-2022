// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterFeederSubsystem extends SubsystemBase {

  private final CANSparkMax m_feederMotor;
  public enum FeederFunctions {
    FORWARD, REVERSE, OFF
  }

  private final WPI_TalonFX m_shooterMotor;

  // current rpm for shooter
  private double m_shooterTargetRPM = 0;
  // is the motor enabled
  private boolean m_shooterEnabled = false;
  // is the shooter at the right rpm
  private int m_timeSpentAtTargetSpeed = 0;
  private static final int PID_IDX = 0;

  private Timer m_shooterCooldownTimer;

  // Math variables needed to convert RPM to ticks per second/ticks per
  private final int SENSOR_CYCLES_PER_SECOND = 10;   // sensor velocity period is 100 ms
  private final int SEC_PER_MIN = 60;
  private final int COUNTS_PER_REV = 2048;

  private static final Logger log = LogManager.getLogger(ShooterFeederSubsystem.class);

  /** Creates a new ShooterSubsystem. */
  public ShooterFeederSubsystem(WPI_TalonFX shooterMotor, CANSparkMax feederMotor)  {
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
    feederMotor.restoreFactoryDefaults();
    feederMotor.setIdleMode(IdleMode.kBrake);
    m_shooterCooldownTimer = new Timer();

    addChild("ShooterMotor", m_shooterMotor);
  } // end constructor


  public void setShooterEnabled (boolean enabled) { 
    if (enabled) {
      m_shooterEnabled = true;
      // System.out.println("enabling shooter");
    } else { // if false
      m_shooterCooldownTimer.start(); // start cooldown timer
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
      //Can do MathUtil clamp -> or set it in setRelativeTargetRPM, then call setRelativeTargetRPM with STEP_CHANGE
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

      //Case OFF not really needed, maybe delete
      case OFF:
        m_feederMotor.set(ShooterConstants.FEEDER_SPEED_OFF);
        break;

      default:
        m_feederMotor.set(ShooterConstants.FEEDER_SPEED_OFF);
        break;
    }
  }
    
  public boolean shooterSpeedIsReady() {
    return m_timeSpentAtTargetSpeed >= ShooterConstants.ITERATIONS_AT_TARGET_RPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_shooterCooldownTimer.hasElapsed(ShooterConstants.SHOOTER_COOLDOWN_TIME))  { // don't turn off shooter until some time has elapsed
      m_shooterEnabled = false;
      m_shooterCooldownTimer.stop();
      m_shooterCooldownTimer.reset();
    }

    if (m_shooterEnabled) {
      double speed = m_shooterTargetRPM * COUNTS_PER_REV / SENSOR_CYCLES_PER_SECOND / SEC_PER_MIN;
      m_shooterMotor.set(ControlMode.Velocity, speed);      
      // System.out.println("target rpm" + m_shooterSpeedTest);
    } else {
      m_shooterMotor.set(0);
      // System.out.println("0");
    }

    if (Math.abs(m_shooterTargetRPM - getRealRPM()) < ShooterConstants.TARGET_RPM_TOLERANCE) {
        m_timeSpentAtTargetSpeed++;
    } else {
      m_timeSpentAtTargetSpeed = 0;
    }

    SmartDashboard.putNumber("shooter real rpm", getRealRPM());
    SmartDashboard.putNumber("shooter target rpm", getTargetRPM());
    SmartDashboard.putNumber("shooter temperature F", (m_shooterMotor.getTemperature() * 9/5) + 32);

    // log.info("shooter real rpm {} shooter target rpm {} ", getRealRPM(), getTargetRPM());
    // System.out.println(m_shooterCooldownTimer.get());
    // System.out.println(m_shooterEnabled);
  }  // END periodic()



} // END class ShooterSubsystem
