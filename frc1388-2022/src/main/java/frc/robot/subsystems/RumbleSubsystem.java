/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RumbleConstants;
import frc.robot.Constants.RumbleConstants.RumbleSide;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleSubsystem extends SubsystemBase {

  private Timer timer = new Timer();

  private XboxController m_controller;
  private RumbleSide m_side = RumbleSide.NONE;

  private int m_numberOfPulses = 0;
  private boolean m_startPulse = false;
  private boolean m_pulseOn = false;
  private boolean m_pulseOff = false;

  /**
   * Creates a new Rumble.
   */
  public RumbleSubsystem( XboxController controller ){
    this.m_controller = controller;
  }

  private void rumble( RumbleSide side, double strength ) {
    for( RumbleType type: side.getRumbleType() ){
      m_controller.setRumble(type, strength);
    }
  } 

  public void rumbleOff(){
    rumble( m_side, RumbleConstants.RUMBLE_OFF);
  }

  public void rumbleOn(){
    rumble( m_side, RumbleConstants.RUMBLE_STRENGTH);
  }

  public void rumblePulse( RumbleSide side){
    m_numberOfPulses = RumbleConstants.NUMBER_OF_PULSES;
    if(m_side != side ){
      rumbleOff();
    }
    m_side = side;
    m_startPulse = true;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if( m_startRumble ){
    //   m_startPulse = true;
    //   m_startRumble = false;
    // }

    if( m_startPulse && m_numberOfPulses > 0 ){
      timer.start();
      rumbleOn();
      m_startPulse = false;
      m_pulseOn = true;
    }

    if( m_pulseOn && timer.hasElapsed(RumbleConstants.RUMBLE_PUSLE_TIME)) {
      rumbleOff();
      timer.reset();
      m_pulseOn = false;
      m_pulseOff = true;
      //TODO edit this?
      m_numberOfPulses--;
    }

    if( m_pulseOff && timer.hasElapsed(RumbleConstants.ANTI_RUMBLE_TIME)){
      m_pulseOff = false;
      m_startPulse = true;
      timer.reset();
    }


    // has time period passed for need to stop
    // has enough pulses occured 
    
  }
}
