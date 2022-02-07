package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class ButtonBinding<T extends Action> extends ActionBinding<T> implements BooleanSupplier {

  public ButtonBinding(T axis, OISubsystem oi, ShuffleboardContainer container) {
    super(axis, oi, container);
  }

  @Override
  public String getChannelName(int channel) {
    var boundJoystick = getBoundController();
    Optional<String> channelName = Optional.empty();
    if (boundJoystick instanceof XboxController) {
      channelName =  Arrays.stream(XboxController.Button.values())
              .filter(btn -> btn.value == channel)
              .findAny()
              .map(Object::toString);
    } else if (boundJoystick instanceof PS4Controller) {
      channelName =  Arrays.stream(PS4Controller.Button.values())
              .filter(btn -> btn.value == channel)
              .findAny()
              .map(Object::toString);
    }
    return channelName.orElse(Integer.toString(channel));
  }

  @Override
  public Integer getSelectedChannel(GenericHID joystick) {
    var buttonField = DriverStation.getStickButtons(joystick.getPort());
    if (buttonField == 0) {
      return null;
    } else {
      // Return log2 rounded down of the bit field plus one.
      // This will extract the position of the highest 1 bit in the field
      // Then we add one since button channel numbers start at 1
      return (int) (Math.log(buttonField) / Math.log(2)) + 1;
    }
  }

  @Override
  public boolean getAsBoolean() {
    return getBoundController().getRawButton(getBoundChannel());
  }
}
