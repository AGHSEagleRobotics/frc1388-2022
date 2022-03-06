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
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DashboardConstants.Cameras;

/** Add your docs here. */
public class Dashboard {

    private final UsbCamera m_frontCamera = CameraServer.startAutomaticCapture(DashboardConstants.FRONT_CAMERA_PORT);
    private final UsbCamera m_reverseCamera = CameraServer.startAutomaticCapture(DashboardConstants.REVERSE_CAMERA_PORT);
    private final UsbCamera m_ballCamera = CameraServer.startAutomaticCapture(DashboardConstants.BALL_CAMERA_PORT);
    // private VideoSource[] m_testVideoSources;
    private final VideoSink m_videoSink = CameraServer.getServer();
    private final VideoSink m_otherVideoSink = CameraServer.getServer();

    private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Cameras");
    private ComplexWidget m_complexWidgetCam;

    //TODO put enum back in here (enum's existance subject to debate)
    private Cameras m_currentCam = Cameras.FORWARDS;

    public Dashboard() { // constructer
        setCamView(Cameras.FORWARDS);
        setupShuffleboard();
    } // end constructer

    private void setupShuffleboard() {
        m_complexWidgetCam = m_shuffleboardTab.add("cams", m_videoSink.getSource())
            .withWidget(BuiltInWidgets.kCameraStream);
        m_otherVideoSink.setSource(m_ballCamera);
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
            case FORWARDS:
                m_videoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            case REVERSE:
                m_videoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            case BALL:
                m_videoSink.setSource(m_ballCamera);
                m_currentCam = Cameras.BALL;
                break;
            default: m_videoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
        }
    }

}
