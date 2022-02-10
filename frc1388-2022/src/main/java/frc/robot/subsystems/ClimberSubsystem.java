// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FalconConstants;

public class ClimberSubsystem extends SubsystemBase {

  private static final Logger log = LogManager.getLogger(ClimberSubsystem.class);

  private final WPI_TalonFX m_winchMotor;
  private final CANSparkMax m_articulatorMotor;

  // private static final double COUNTS_PER_REV = 2048.0;
    /** winch gearbox ratio */
  private static final double WINCH_GEAR_RATIO = 20.0;
    /** winch diamater in inches */ 
  private static final double WINCH_DIAMATER = 2.0; 
    /** sensor position / WINCH_SENSOR_UNITS_PER_INCH = arm extention in inches */
  private static final double WINCH_SENSOR_UNITS_PER_INCH = FalconConstants.COUNTS_PER_REV * WINCH_GEAR_RATIO / (WINCH_DIAMATER * Math.PI);
    /** length of winch arm in inches */
  private static final double WINCH_ARM_LENGTH = 24;

  private static final int PID_IDX = 0;
  
    
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(WPI_TalonFX winchMotor, CANSparkMax articulatorMotor) {
    m_winchMotor = winchMotor;
    m_articulatorMotor = articulatorMotor;

    // setting defaults and nutral mode to break
    m_winchMotor.configFactoryDefault();
    m_winchMotor.setNeutralMode(NeutralMode.Brake);
    m_winchMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);      // todo test this
    m_winchMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);      // todo test this
    m_winchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_winchMotor.setSensorPhase(false);
    m_winchMotor.setInverted(false);
    m_winchMotor.setSelectedSensorPosition(0);
    m_winchMotor.configReverseSoftLimitThreshold(WINCH_SENSOR_UNITS_PER_INCH * WINCH_ARM_LENGTH);
    m_winchMotor.configPeakOutputForward(ClimberConstants.CLIMBER_MAX_POWER_FORWARDS);
    m_winchMotor.configPeakOutputReverse(ClimberConstants.CLIMBER_MAX_POWER_REVERSE);

    m_winchMotor.config_kF(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_F);
    m_winchMotor.config_kP(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_P);
    m_winchMotor.config_kI(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_I);
    m_winchMotor.config_kD(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_D);


    m_articulatorMotor.setInverted(false);
    /* for talonFX / talonSRX */
    // m_articulatorMotor.configFactoryDefault();
    // m_articulatorMotor.setNeutralMode(NeutralMode.Brake);
    // m_articulatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen); // todo test this
    // m_articulatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen); // todo test this
    // m_articulatorMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    // m_articulatorMotor.setSensorPhase(false);
    // m_articulatorMotor.setInverted(false);
    // m_articulatorMotor.setSelectedSensorPosition(0);
    // m_articulatorMotor.configPeakOutputForward(ClimberConstants.ARTICULATOR_MAX_POWER_FORWARDS);
    // m_articulatorMotor.configPeakOutputReverse(ClimberConstants.ARTICULATOR_MAX_POWER_REVERSE);

    // m_articulatorMotor.config_kF(PID_IDX, ClimberConstants.ARTICULATOR_GAINS_POSITION_F);
    // m_articulatorMotor.config_kP(PID_IDX, ClimberConstants.ARTICULATOR_GAINS_POSITION_P);
    // m_articulatorMotor.config_kI(PID_IDX, ClimberConstants.ARTICULATOR_GAINS_POSITION_I);
    // m_articulatorMotor.config_kD(PID_IDX, ClimberConstants.ARTICULATOR_GAINS_POSITION_D);

  } 

  /** set power
   * @param power power from [-1, 1]
  */
  public void setWinchPower (double power) {
    m_winchMotor.set(power);
  }
  
    /** set speed
     * @param speed speed in inches per second
     */
  public void setWinchSpeed (double speed) {
    double velocity = speed * WINCH_SENSOR_UNITS_PER_INCH / FalconConstants.SENSOR_CYCLES_PER_SECOND;
    m_winchMotor.set(ControlMode.Velocity, velocity);
  }

  /** returns inches */
  public double getWinchPossition() {
    return m_winchMotor.getSelectedSensorPosition() / WINCH_SENSOR_UNITS_PER_INCH;
  }

  public boolean winchAtBottomLimit() {
    return (m_winchMotor.isFwdLimitSwitchClosed() == 1); // todo test this
  }

  public void setArticulatorPower (double power) {
    m_articulatorMotor.set(power);
  }

  public void setArticulatorPosition (double position) {
    // m_articulatorMotor.setVoltage(position); // m_articulatorMotor.set(ControlMode.Position, position);  // todo convertion 
  }

/*
  public void toggleArticuilatorPosition (boolean on) {
    if (on) {
      m_winchMotor.set(ControlMode.Position,  2000);
    } else {
      m_winchMotor.set(ControlMode.Position,  0);
    }
  }
  */

  /** returns sensor units */
  public double getArticulatorPossition() {
    return 123456789; // m_articulatorMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log.debug("winch postition {}", this::getWinchPossition);

    if (winchAtBottomLimit()) {
      m_winchMotor.setSelectedSensorPosition(0);
    }

  }
}
