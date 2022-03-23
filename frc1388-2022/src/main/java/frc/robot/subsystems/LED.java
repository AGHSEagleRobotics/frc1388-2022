// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.pattern.AbstractStyleNameConverter.Blue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static final Logger log = LogManager.getLogger(LED.class);


  private PWMSparkMax m_LedArms;
  private PWMSparkMax m_LedBody;
  /**tells wether the led is set */
  private boolean m_isLedSet = false;
  private String m_setTeam = "blue";
  private String m_getTeam;
  // private double m_value = -0.99;
  // private final double m_change = 0.02;

  /** Creates a new LED. */
  public LED(PWMSparkMax armLeds, PWMSparkMax bodyLeds) {
    m_LedArms = armLeds;
    m_LedBody = bodyLeds;
  } // end constructor

  /**sets the led value.
  //  * @param num clamps from -0.99 to 0.99  */
  // public void setLED(double num) {
  //   MathUtil.clamp(num, -0.99, 0.99);
  //   m_LedArms.set(num);
  // }
  
  /**change led up */
  // public void increaseLED() {
  //   m_value += m_change;
  //   MathUtil.clamp(m_value, -0.99, 0.99);
  //   m_LedArms.set(m_value);
  // }

  /**change led down */
  // public void decreaseLED() {
  //   m_value -= m_change;
  //   MathUtil.clamp(m_value, -0.99, 0.99);
  //   m_LedArms.set(m_value);
  // }

  // public double getValue() {
  //   return m_value;
  // }

  public void ledShoot() {
    if (m_getTeam == "blue") {
      m_LedBody.set(-0.83);
    } else {
      m_LedBody.set(-0.85);
    }
  }

  public void ledNormal() {
    if (m_getTeam == "blue") {
      m_LedBody.set(0.87);
    } else {
      m_LedBody.set(0.61);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // log.info("led value: {}", getValue());
    if (DriverStation.isFMSAttached() && !m_isLedSet) {
      m_isLedSet = true;
      // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      if (m_setTeam == "blue") { 
        m_getTeam = "blue";
        m_LedArms.set(-0.01); // FIXME set in led controler
        m_LedBody.set(0.87);
      } else {
        m_getTeam = "red";
        m_LedArms.set(-0.19); // FIXME set in led controler
        m_LedBody.set(0.61);
      }
    }

  }
}
