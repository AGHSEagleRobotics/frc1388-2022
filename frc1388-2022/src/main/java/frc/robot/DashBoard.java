// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;

/** Add your docs here. */
public class DashBoard {

    private UsbCamera m_testCamera;
    // private VideoSource[] m_testVideoSources;
    private VideoSink m_testVideoSink;


    public DashBoard() {
        camSetup();
    }

    public void camSetup() {
        m_testCamera = CameraServer.startAutomaticCapture(0);
        // m_testVideoSources = new VideoSource[] { 
            // m_testCamera 
        // };

        m_testVideoSink = CameraServer.getServer();
        m_testVideoSink.setSource(m_testCamera);

    }
}
