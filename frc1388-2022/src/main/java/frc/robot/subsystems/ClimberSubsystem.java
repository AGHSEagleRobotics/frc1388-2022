// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.DigestInputStream;

import javax.swing.filechooser.FileNameExtensionFilter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANDigitalInput;
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
import frc.robot.Constants.ClimberCommandConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FalconConstants;

public class ClimberSubsystem extends SubsystemBase {
  
  private static final Logger log = LogManager.getLogger(ClimberSubsystem.class);
  
  private final WPI_TalonFX m_winchMotor;
    // private DigitalInput m_winchLimit;
    // private boolean m_isWinchReset = false;
  private final CANSparkMax m_articulatorMotor;
    private final SparkMaxLimitSwitch m_artuculatorForwardsLimit;
    private final SparkMaxLimitSwitch m_articulatorReverseLimit;
  
  // private final RelativeEncoder m_articulatorEncoder;

  // private static final double COUNTS_PER_REV = 2048.0;
    /** winch gearbox ratio */
  private static final double WINCH_GEAR_RATIO = 20.0;
    /** winch diamater in inches */ 
  private static final double WINCH_DIAMETER = 2.0; 
    /** sensor position / WINCH_SENSOR_UNITS_PER_INCH = arm extention in inches */
  private static final double WINCH_SENSOR_UNITS_PER_INCH = FalconConstants.COUNTS_PER_REV * WINCH_GEAR_RATIO / (WINCH_DIAMETER * Math.PI);
    /** length of winch arm in inches */
  private static final double WINCH_ARM_LENGTH = 24;

  // private static final int PID_IDX = 0;

  // private SparkMaxPIDController m_articulatorPidController;
  // private SparkMaxLimitSwitch m_articulatorVerticalLimitSwitch;

  // private DigitalInput m_reverseArticulatorLimit;
  // private DigitalInput m_forwardsArticulatorLimit;

  // private final SparkMaxLimitSwitch m_forwardLimitSwitch;
  // private final SparkMaxLimitSwitch m_reverseLimitSwitch;

  private final DigitalInput m_winchForward;
  private final DigitalInput m_winchReverse;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(WPI_TalonFX winchMotor, CANSparkMax articulatorMotor, DigitalInput winchForward, DigitalInput winchReverse) { // constructer

    m_winchMotor = winchMotor;

      // m_winchLimit = winchLimit;
      m_winchMotor.configFactoryDefault();
      m_winchMotor.setNeutralMode(NeutralMode.Brake);
      m_winchMotor.setInverted(false);
      // m_winchMotor.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalon, LimitSwitchNormal.NormallyClosed);
      // m_winchMotor.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalon, LimitSwitchNormal.NormallyClosed);
      m_winchForward = winchForward;
      m_winchReverse = winchForward;

    m_articulatorMotor = articulatorMotor;

      m_articulatorMotor.restoreFactoryDefaults();
      m_articulatorMotor.setInverted(false);
      m_articulatorMotor.setIdleMode(IdleMode.kBrake);
      m_articulatorMotor.setSecondaryCurrentLimit(ClimberConstants.ARTICULATOR_MAX_SMART_CURRENT_LIMIT);
      m_artuculatorForwardsLimit = m_articulatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      m_articulatorReverseLimit = m_articulatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);



    // setting defaults and nutral mode to break

    //        m_winchMotor.setSensorPhase(false);
    //        m_winchMotor.setInverted(false);
    //        m_winchMotor.setSelectedSensorPosition(0);
    //        m_winchMotor.configForwardSoftLimitThreshold(1000000); // FIXME add constant & actual number
    //        m_winchMotor.configForwardSoftLimitEnable(true);
    // m_winchMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);      // todo test this
    // m_winchMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);      // todo test this
    // m_winchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    // m_winchMotor.configPeakOutputForward(ClimberConstants.CLIMBER_MAX_POWER_FORWARDS);
    // m_winchMotor.configPeakOutputReverse(ClimberConstants.CLIMBER_MAX_POWER_REVERSE);

    // m_winchMotor.config_kF(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_F);
    // m_winchMotor.config_kP(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_P);
    // m_winchMotor.config_kI(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_I);
    // m_winchMotor.config_kD(PID_IDX, ClimberConstants.WINCH_GAINS_VELOCITY_D);

    // =========== //
    // articulator //
    // =========== //
     //setNeutralMode(NeutralMode.Brake);
    // m_articulatorMotor.
    // m_articulatorEncoder = m_articulatorMotor.getEncoder();
    // m_articulatorPidController = m_articulatorMotor.getPIDController();

    // m_articulatorMotor.setSoftLimit     (CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.ARTIUCLATOR_REACH_SOFT_LIMIT);
    // m_articulatorMotor.setSoftLimit     (CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.ARTIUCLATOR_VERTICAL_SOFT_LIMIT);
    // m_articulatorMotor.enableSoftLimit  (CANSparkMax.SoftLimitDirection.kForward, true);
    // m_articulatorMotor.enableSoftLimit  (CANSparkMax.SoftLimitDirection.kReverse, true);
    

    // m_articulatorVerticalLimitSwitch = m_articulatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // m_articulatorVerticalLimitSwitch.enableLimitSwitch(true);

    // m_articulatorPidController.setP  (ClimberConstants.ARTICULATOR_GAINS_POSITION_P);
    // m_articulatorPidController.setI  (ClimberConstants.ARTICULATOR_GAINS_POSITION_I);
    // m_articulatorPidController.setD  (ClimberConstants.ARTICULATOR_GAINS_POSITION_D);
    // m_articulatorPidController.setFF (ClimberConstants.ARTICULATOR_GAINS_POSITION_F);

    //FIXME Make forward instead of reverse, clarify "reverse" is towards the "front" - ?
    // m_reverseArticulatorLimit = new DigitalInput(3);
    // m_forwardsArticulatorLimit = new DigitalInput(4);
  } 

  /** set power
   * @param power power from [-1, 1]
  */
  public void setWinchPower (double power) {
    if ((power == 0) || (m_winchForward.get() && power > 0) || (m_winchReverse.get() && power < 0)) {
      m_winchMotor.set(power * ClimberConstants.CLIMBER_MAX_POWER_FORWARDS);
    }
  }

  // articulator
  public void setArticulatorPower (double power) {
    m_articulatorMotor.set(power * ClimberConstants.ARTICULATOR_MAX_POWER_FORWARDS); // keep
  }

  // public void resetWinchLimit() {
  //   if (!m_isWinchReset && !m_winchLimit.get()) {
  //     m_winchMotor.set(-0.3);
  //   } else {
  //     m_isWinchReset = true;
  //     m_winchMotor.setSelectedSensorPosition(0);
  //     m_winchMotor.set(0);
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
