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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FalconConstants;
import frc.robot.Constants.ClimberConstants.ArticulatorPositions;

public class ClimberSubsystem extends SubsystemBase {

  private static final Logger log = LogManager.getLogger(ClimberSubsystem.class);

  private final WPI_TalonFX m_winchMotor;
  private final CANSparkMax m_articulatorMotor;
  private final RelativeEncoder m_articulatorEncoder;

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

  private boolean m_articulatorIsMoving = false;
  private ArticulatorPositions m_articulatorPosition;

  private SparkMaxPIDController m_articulatorPidController;
  private SparkMaxLimitSwitch m_articulatorVerticalLimitSwitch;

  private DigitalInput m_reverseArticulatorLimitSwitch;

  
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

    // =========== //
    // articulator //
    // =========== //
    m_articulatorMotor.restoreFactoryDefaults();
    m_articulatorMotor.setInverted(false);
    m_articulatorMotor.setIdleMode(IdleMode.kBrake); //setNeutralMode(NeutralMode.Brake);
    m_articulatorEncoder = m_articulatorMotor.getEncoder();
    m_articulatorPidController = m_articulatorMotor.getPIDController();

    m_articulatorMotor.enableSoftLimit  (CANSparkMax.SoftLimitDirection.kForward, true);
    m_articulatorMotor.enableSoftLimit  (CANSparkMax.SoftLimitDirection.kReverse, true);
    m_articulatorMotor.setSoftLimit     (CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.ARTIUCLATOR_REACH_SOFT_LIMIT);
    m_articulatorMotor.setSoftLimit     (CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.ARTIUCLATOR_VERTICAL_SOFT_LIMIT);

    m_articulatorVerticalLimitSwitch = m_articulatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_articulatorVerticalLimitSwitch.enableLimitSwitch(true);

    m_articulatorPidController.setP  (ClimberConstants.ARTICULATOR_GAINS_POSITION_P);
    m_articulatorPidController.setI  (ClimberConstants.ARTICULATOR_GAINS_POSITION_I);
    m_articulatorPidController.setD  (ClimberConstants.ARTICULATOR_GAINS_POSITION_D);
    m_articulatorPidController.setFF (ClimberConstants.ARTICULATOR_GAINS_POSITION_F);

    m_reverseArticulatorLimitSwitch = new DigitalInput(3);


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

  // articulator
  public void setArticulatorPower (double power) {
    m_articulatorMotor.set(power);
  }

  public void setArticulatorVertical () {
    m_articulatorPidController.setReference(ArticulatorPositions.VERTICAL.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public void setArticulatorReach () {
    m_articulatorPidController.setReference(ArticulatorPositions.REACH.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public boolean getArticulatorVerticalLimit() {
    return m_reverseArticulatorLimitSwitch.get();
  }

  public boolean articulatorAtVerticalLimit() {
    return m_articulatorVerticalLimitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log.debug("winch postition {}", this::getWinchPossition);
    // log.debug("Articulator target possition {}", this::getArticulatorTargetPossition);
    log.debug("Articulator possition {} ", m_articulatorEncoder::getPosition);
    SmartDashboard.putNumber("Articulator possition", m_articulatorEncoder.getPosition());

    if (winchAtBottomLimit()) {
      m_winchMotor.setSelectedSensorPosition(0);
      
    }

    if (getArticulatorVerticalLimit()) {
      m_articulatorEncoder.setPosition(0);
      // setArticulatorVertical();
      // setArticulatorPower(0);
    }
    SmartDashboard.putBoolean("DIO4", m_reverseArticulatorLimitSwitch.get());

  }
}
