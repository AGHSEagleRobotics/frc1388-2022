// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeArmMotor;
  private final CANSparkMax m_intakeWheelSpin;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(CANSparkMax intakeArmMotor, CANSparkMax intakeWheelSpin) {
    m_intakeArmMotor = intakeArmMotor;
    m_intakeWheelSpin = intakeWheelSpin;

    m_intakeArmMotor.restoreFactoryDefaults();
    m_intakeArmMotor.setIdleMode(IdleMode.kCoast);

    m_intakeWheelSpin.restoreFactoryDefaults();
    m_intakeWheelSpin.setIdleMode(IdleMode.kCoast);
   }

   public void setIntakeWheelSpin(double speed){
     m_intakeWheelSpin.set(speed);
   }

   public void setIntakeArmMotor(double speed){
    m_intakeArmMotor.set(speed);
    //TODO actually use this
    m_intakeArmMotor.getForwardLimitSwitch(Type.kNormallyClosed);
    m_intakeArmMotor.getReverseLimitSwitch(Type.kNormallyClosed);
   }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
