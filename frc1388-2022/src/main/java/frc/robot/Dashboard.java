// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DashboardConstants.Cameras;

/** Add your docs here. */
public class Dashboard {

    private final UsbCamera m_frontCamera = CameraServer.startAutomaticCapture(0);
    private final UsbCamera m_reverseCamera = CameraServer.startAutomaticCapture(1);
    // private VideoSource[] m_testVideoSources;
    private final VideoSink m_testVideoSink = CameraServer.getServer();

    private final ShuffleboardTab m_shuffelboardTab = Shuffleboard.getTab("Cameras");
    private ComplexWidget m_complexWidgetCam;

    private Cameras m_currentCam = Cameras.FOREWARDS;

    public Dashboard() { // constructer
        setCamView(Cameras.FOREWARDS);
        setupShuffelboard();
    } // end constructer

    private void setupShuffelboard() {
        m_complexWidgetCam = m_shuffelboardTab.add("cams", m_testVideoSink.getSource())
            .withWidget(BuiltInWidgets.kCameraStream);
    }

    public void switchCamera() {
        System.out.println("swich camera method");
        switch (m_currentCam) {
            case FOREWARDS: m_testVideoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            case REVERSE: m_testVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FOREWARDS;
                break;
            default: m_testVideoSink.setSource(m_reverseCamera);
        }

        // if (m_currentCam == Cameras.forewards) {
        //     m_testVideoSink.setSource(m_reverseCamera);
        //     // System.out.println("test camera");
        // } else {
        //     m_testVideoSink.setSource(m_frontCamera);
        //     // System.out.println("other test camera");            
        // }
    }

    public void setCamView(Cameras camera) {
        switch (camera) {
            case FOREWARDS: m_testVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FOREWARDS;
                break;
            case REVERSE: m_testVideoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            default: m_testVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FOREWARDS;
        }
    }

}
