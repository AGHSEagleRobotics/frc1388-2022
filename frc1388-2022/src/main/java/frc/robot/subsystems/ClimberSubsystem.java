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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_winchMotor;
  private final WPI_TalonSRX m_articulatorMotor;

  private double m_winchMotorPower = 0;
  private double m_articulatorMotorPower = 0;

  private static final double COUNTS_PER_REV = 2048.0;
    /** winch gearbox ratio */
  private static final double WINCH_GEAR_RATIO = 20.0;
    /** winch diamater in inches */ 
  private static final double WINCH_DIAMATER = 2.0;    


  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(WPI_TalonFX winchMotor, WPI_TalonSRX articulatorMotor) {
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

    m_articulatorMotor.configFactoryDefault();
    m_articulatorMotor.setNeutralMode(NeutralMode.Brake);
    m_articulatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen); // todo test this
    m_articulatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen); // todo test this
    m_articulatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_articulatorMotor.setSensorPhase(false);
    m_articulatorMotor.setInverted(false);
    m_articulatorMotor.setSelectedSensorPosition(0);   

  } 

  public void setWinchPower (double power) {
    m_winchMotorPower = power;
  }

  /** returns inches */
  public double getWinchPossition() {
    return m_winchMotor.getSelectedSensorPosition() / COUNTS_PER_REV / WINCH_GEAR_RATIO * WINCH_DIAMATER * Math.PI;
  }

  public void setArticulatorPower (double power) {
    m_articulatorMotorPower = power;
  }

  /** returns sensor units */
  public double getArticulatorPossition() {
    return m_articulatorMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    m_winchMotor.set(m_winchMotorPower);
    m_articulatorMotor.set(m_articulatorMotorPower);
    
    System.out.println(getWinchPossition());


  }
}
