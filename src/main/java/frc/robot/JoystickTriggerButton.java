package frc.robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class JoystickTriggerButton extends Button {
    private final GenericHID m_joystick;
    private final int m_axisNumber;
    private final double m_triggerValue;

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     * @param triggerValue The value needed to pass for the button to be considered pressed. 
     *                          (LT/RT triggers return 0 to 1 so somewhere in that range)
     */
    public JoystickTriggerButton(GenericHID joystick, int axisNumber, double triggerValue) {
        requireNonNullParam(joystick, "joystick", "JoystickButton");

        m_joystick = joystick;
        m_axisNumber = axisNumber;
        m_triggerValue = triggerValue;
    }

    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
        return m_joystick.getRawAxis(m_axisNumber) > m_triggerValue;
    }
}
