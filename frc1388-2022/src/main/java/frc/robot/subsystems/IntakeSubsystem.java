// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeArmMotor;
  private final CANSparkMax m_intakeWheelSpin;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(CANSparkMax intakeArmMotor, CANSparkMax intakeWheelSpin) {
    m_intakeArmMotor = intakeArmMotor;
    m_intakeWheelSpin = intakeWheelSpin;
   }

   public void setIntakeWheelSpin(double speed){
     m_intakeWheelSpin.set(speed);
   }

   public void setIntakeArmMotor(double speed){
    m_intakeArmMotor.set(speed);
   }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
