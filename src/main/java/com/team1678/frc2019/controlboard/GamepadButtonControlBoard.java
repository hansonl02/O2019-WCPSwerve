package com.team1678.frc2019.controlboard;

import com.team1678.frc2019.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private XboxController mJoystick;

    private GamepadButtonControlBoard() {
        mJoystick = new XboxController(Constants.kButtonGamepadPort);
    }

    // Cargo Intake
    @Override
    public boolean getRunIntake() {
        return mJoystick.getTriggerAxis(Hand.kRight) > Constants.kJoystickThreshold;
    }
    
    @Override
    public boolean getRunOuttake() {
        return mJoystick.getTriggerAxis(Hand.kLeft) > Constants.kJoystickThreshold;
    }

    @Override
    public boolean getScoreHatch() {
        return mJoystick.getBumper(Hand.kLeft);
    }

    @Override
    public void setRumble(boolean on) {
        mJoystick.setRumble(RumbleType.kRightRumble, on ? 1.0 : 0.0);
    }
}
