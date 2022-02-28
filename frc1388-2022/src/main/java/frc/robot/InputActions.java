package frc.robot;

import com.eaglerobotics.lib.shuffleboard.InputAction;
import edu.wpi.first.wpilibj.XboxController;

public final class InputActions {
  public enum AxisInputs implements InputAction {
    DRIVE_SPEED("Drive Speed", 0, XboxController.Axis.kLeftY.value),
    DRIVE_TURN("Drive Turn Speed", 0, XboxController.Axis.kRightX.value),
    CLIMBER_EXTEND("Extend/Retract Climber", 1, XboxController.Axis.kLeftY.value),
    CLIMBER_ARTICULATE("Articulate Climber", 1, XboxController.Axis.kRightY.value),
    ;

    private final String label;
    private final int defaultPort;
    private final int defaultChannel;

    AxisInputs(String label, int defaultPort, int defaultChannel) {
      this.label = label;
      this.defaultPort = defaultPort;
      this.defaultChannel = defaultChannel;
    }

    @Override
    public String getLabel() {
      return label;
    }

    @Override
    public String getDescription() {
      return null;
    }

    @Override
    public Integer getDefaultChannel() {
      return defaultChannel;
    }

    @Override
    public Integer getDefaultPort() {
      return defaultPort;
    }
  }

  public enum ButtonInputs implements InputAction {
    RETRACT_INTAKE_DRIVER("Retract Intake (driver)", 0, XboxController.Button.kLeftBumper.value),
    RETRACT_INTAKE_OP("Retract Intake (op)", 1, XboxController.Button.kLeftBumper.value),
    SHOOT_HIGH("Shoot High", 0, XboxController.Button.kRightBumper.value),
    CLIMBER_REACH("Climber Reach", 1, XboxController.Button.kX.value),
    CLIMBER_STOW("Climber Stow", 1, XboxController.Button.kY.value),
    DRIVE_REVERSE("Drive Mode - Reverse", 0, XboxController.Button.kA.value),
    DRIVE_FORWARD("Drive Mode - Forward", 1, XboxController.Button.kB.value),
    DRIVE_PRECISION_MODE("Precision Mode", 0, XboxController.Button.kRightStick.value),
    CYCLE_CAMERA_DRIVER("Cycle Cameras (driver)", 0, XboxController.Button.kStart.value),
    CYCLE_CAMERA_OP("Cycle Cameras (op)", 0, XboxController.Button.kStart.value),
    ;

    private final String label;
    private final int defaultPort;
    private final int defaultChannel;

    ButtonInputs(String label, int defaultPort, int defaultChannel) {
      this.label = label;
      this.defaultPort = defaultPort;
      this.defaultChannel = defaultChannel;
    }

    @Override
    public String getLabel() {
      return label;
    }

    @Override
    public String getDescription() {
      return null;
    }

    @Override
    public Integer getDefaultChannel() {
      return defaultChannel;
    }

    @Override
    public Integer getDefaultPort() {
      return defaultPort;
    }
  }
}
