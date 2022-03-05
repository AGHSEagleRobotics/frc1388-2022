// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
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
    private final VideoSink m_videoSink = CameraServer.getServer();

    private final ShuffleboardTab m_shuffelboardTab = Shuffleboard.getTab("Cameras");
    private ComplexWidget m_complexWidgetCam;

    //TODO put enum back in here (enum's existance subject to debate)
    private Cameras m_currentCam = Cameras.FORWARDS;

    public Dashboard() { // constructer
        setCamView(Cameras.FORWARDS);
        setupShuffelboard();
    } // end constructer

    private void setupShuffelboard() {
        m_complexWidgetCam = m_shuffelboardTab.add("cams", m_videoSink.getSource())
            .withWidget(BuiltInWidgets.kCameraStream);
    }

    public void switchCamera() {
        System.out.println("swich camera method");
        switch (m_currentCam) {
            case FORWARDS: m_videoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            case REVERSE: m_videoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            default: m_videoSink.setSource(m_reverseCamera);
        }
    }

    //This is sort of duplicated code
    public void setCamView(Cameras camera) {
        switch (camera) {
            case FORWARDS: m_videoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            case REVERSE: m_videoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            default: m_videoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
        }
    }

}
