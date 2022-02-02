// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_winchMotor;
  private final WPI_TalonSRX m_articulatorMotor;

  private double m_winchMotorPower = 0;
  private double m_articulatorMotorPower = 0;

  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(WPI_TalonFX winchMotor, WPI_TalonSRX articulatorMotor) {
    m_winchMotor = winchMotor;
    m_articulatorMotor = articulatorMotor;

    // setting defaults and nutral mode to break
    m_winchMotor.configFactoryDefault();
    m_winchMotor.setNeutralMode(NeutralMode.Brake);

    m_articulatorMotor.configFactoryDefault();
    m_articulatorMotor.setNeutralMode(NeutralMode.Brake);

  } 

  public void setWinchPower (double power) {
    m_winchMotorPower = power;
  }

  public void setArticulatorPower (double power) {
    m_articulatorMotorPower = power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    m_winchMotor.set(m_winchMotorPower);
    m_articulatorMotor.set(m_articulatorMotorPower);
    


  }
}
