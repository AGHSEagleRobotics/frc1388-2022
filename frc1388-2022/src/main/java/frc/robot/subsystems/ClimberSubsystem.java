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
import com.revrobotics.CANSparkMax.IdleMode;

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

    m_articulatorMotor.setSoftLimit     (CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.ARTIUCLATOR_REACH_SOFT_LIMIT);
    m_articulatorMotor.setSoftLimit     (CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.ARTIUCLATOR_VERTICAL_SOFT_LIMIT);
    m_articulatorMotor.enableSoftLimit  (CANSparkMax.SoftLimitDirection.kForward, true);
    m_articulatorMotor.enableSoftLimit  (CANSparkMax.SoftLimitDirection.kReverse, true);
    

    m_articulatorVerticalLimitSwitch = m_articulatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_articulatorVerticalLimitSwitch.enableLimitSwitch(true);

    m_articulatorPidController.setP  (ClimberConstants.ARTICULATOR_GAINS_POSITION_P);
    m_articulatorPidController.setI  (ClimberConstants.ARTICULATOR_GAINS_POSITION_I);
    m_articulatorPidController.setD  (ClimberConstants.ARTICULATOR_GAINS_POSITION_D);
    m_articulatorPidController.setFF (ClimberConstants.ARTICULATOR_GAINS_POSITION_F);

    //Make forward instead of reverse, clarify "reverse" is towards the "front" FIXME
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
  public double getWinchPosition() {
    return m_winchMotor.getSelectedSensorPosition() / WINCH_SENSOR_UNITS_PER_INCH;
  }

  public boolean isWinchAtBottomLimit() {
    return (m_winchMotor.isFwdLimitSwitchClosed() == 1); // TODO test this
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

  public boolean isArticulatorAtVerticalLimit() {
    return m_reverseArticulatorLimitSwitch.get();
  }

  // low priority
  public void setArticulatorPosition (ArticulatorPositions position) {
    m_articulatorPosition = position;
    m_articulatorIsMoving = true;
    
    // switch (position) {
    //   case VERTICAL:
    //     m_articulatorIsMoving = true;
    //     setArticulatorPower(0.5);
    //     break;
    //   case REACH:
    //     m_articulatorIsMoving = true;
    //     setArticulatorPower(-0.5);
    //     break;
    
    //   default:
    //     break;
    // }
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
  
  // low priority
  /** returns sensor units */
  public double getArticulatorTargetPossition() {
    return m_articulatorEncoder.getPosition() * 42; // TODO change 42 to sensor units per rev
    // return m_articulatorEncoder.getVelocity();
  } 

  // low priority
  public int getArticulatroPositionToTarget() {
    if((getArticulatorTargetPossition() <= m_articulatorPosition.getPosition() + 5) && (getArticulatorTargetPossition() >= m_articulatorPosition.getPosition() - 5)) {
      return ClimberConstants.ARTICULATOR_IN_RANGE;
    } else if (getArticulatorTargetPossition() < m_articulatorPosition.getPosition() - 5) {
      return ClimberConstants.ARTICULATOR_BELOW_RANGE;
    } else if (getArticulatorTargetPossition() > m_articulatorPosition.getPosition() + 5) {
      return ClimberConstants.ARTICULATOR_ABOVE_RANGE;
    } 
    return ClimberConstants.ARTICULATOR_IN_RANGE;
  }

  public boolean articulatorAtVerticalLimit() {
    return m_articulatorVerticalLimitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO check for limit swich

  // low priority
    if (m_articulatorIsMoving) {
      // check if it is at the target
      //if(getArticulatorPossition() >= m_articulatorPosition.getPosition())
      if(getArticulatroPositionToTarget() == ClimberConstants.ARTICULATOR_IN_RANGE) {
        setArticulatorPower(0);
        m_articulatorIsMoving = false;
      } else if (getArticulatroPositionToTarget() == ClimberConstants.ARTICULATOR_ABOVE_RANGE) {
        setArticulatorPower(-0.1);
      } else if (getArticulatroPositionToTarget() == ClimberConstants.ARTICULATOR_BELOW_RANGE) {
        setArticulatorPower(0.1);
      }
      // if it is, turn the power to 0
    }
    log.debug("winch postition {}", this::getWinchPosition);
    log.debug("Articulator target possition {}", this::getArticulatorTargetPossition);
    log.debug("Articulator possition {} ", m_articulatorEncoder::getPosition);
    SmartDashboard.putNumber("Articulator possition", m_articulatorEncoder.getPosition());

    if (isWinchAtBottomLimit()) {
      m_winchMotor.setSelectedSensorPosition(0);
      
    }

    //FIXME
    if (isArticulatorAtVerticalLimit()) {
      m_articulatorEncoder.setPosition(0);
      // setArticulatorVertical();
      // setArticulatorPower(0);
    }
    //Update - The name should be what it is, change - also review limit switches TODO
    SmartDashboard.putBoolean("DIO4", m_reverseArticulatorLimitSwitch.get());

  }
}
