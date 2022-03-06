// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DashboardConstants.Cameras;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class Dashboard {
    private ShuffleboardTab m_shuffleboardTab;


    private static SendableChooser<Position> m_autoPosition = new SendableChooser<>();
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();
    private UsbCamera m_cameraColor;

    //FIXME CHANGE THESE to constants? 
    private ComplexWidget complexWidgetAuton;
    private final int autonChooserWidth = 8;
    private final int autonChooserHeight = 2;
    private final int autonChooserColumnIndex = 0;
    private final int autonChooserRowIndex = 0;


 private final UsbCamera m_frontCamera = CameraServer.startAutomaticCapture(0);
    private final UsbCamera m_reverseCamera = CameraServer.startAutomaticCapture(1);
    // private VideoSource[] m_testVideoSources;
    private final VideoSink m_testVideoSink = CameraServer.getServer();

    private final ShuffleboardTab m_shuffelboardTab = Shuffleboard.getTab("Cameras");
    private ComplexWidget m_complexWidgetCam;

    //TODO put enum back in here (enum's existance subject to debate)
    private Cameras m_currentCam = Cameras.FORWARDS;

    public Dashboard() { // constructer
        setCamView(Cameras.FORWARDS);
        setupShuffleboard();
        colorcamera();
        shuffleboardSetUp();
    } // end constructer

    public enum Position {
        POSITION1 ("POSITION1"),
        POSITION2 ("POSITION2"),
        POSITION3 ("POSITION3"),
        POSITION4 ("POSITION4");

        public static final Position Default = POSITION1;

        private String name;

        private Position (String m_name) {
            name = m_name; 
        }
        public String getName(){
            return name;
        }
    }

    public enum Objective {

        LEAVETARMAC ("Leave Tarmac"), //FIXME change these distances (or delete)
        SHOOTBALL1 ("Shoot Starter Ball"),
        PICKUPSHOOT2 ("Pick Up To Left, Shoot"),
        DONOTHING ("Does nothing");

        public static final Objective Default = LEAVETARMAC;

        private String name;

        private Objective (String m_name) {
            m_name = name;
        }
        public String getName(){
            return name;
        }

    }

    public Position getPosition() {
        return m_autoPosition.getSelected();
    }

    public Objective getObjective() {
        return m_autoObjective.getSelected();
    }

    private void colorcamera() {
        m_cameraColor = CameraServer.startAutomaticCapture(AutoConstants.USB_CAMERACOLOR);
    }

    public void shuffleboardSetUp() {
        m_shuffleboardTab =  Shuffleboard.getTab("Competition");
        Shuffleboard.selectTab("Competition");
        for ( Objective o: Objective.values()) {
            m_autoObjective.addOption(o.getName(), o);
        }

        complexWidgetAuton = Shuffleboard.getTab("Competition").add( "AutonChooser", m_autoObjective)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(autonChooserWidth, autonChooserHeight)
            .withPosition(autonChooserColumnIndex, autonChooserRowIndex);
    }

    private void setupShuffleboard() {
        m_complexWidgetCam = m_shuffelboardTab.add("cams", m_testVideoSink.getSource())
            .withWidget(BuiltInWidgets.kCameraStream);
    }

    public void switchCamera() {
        System.out.println("swich camera method");
        switch (m_currentCam) {
            case FORWARDS: m_testVideoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            case REVERSE: m_testVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            default: m_testVideoSink.setSource(m_reverseCamera);
        }
    }

    //This is sort of duplicated code
    public void setCamView(Cameras camera) {
        switch (camera) {
            case FORWARDS: m_testVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
                break;
            case REVERSE: m_testVideoSink.setSource(m_reverseCamera);
                m_currentCam = Cameras.REVERSE;
                break;
            default: m_testVideoSink.setSource(m_frontCamera);
                m_currentCam = Cameras.FORWARDS;
        }
    }

}
