package frc.lib.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSteamInputController extends CommandXboxController {
    private static final double STICK_MULTIPLIER = 1 / 0.75;

    public CommandSteamInputController(int port) {
        super(port);
    }

    @Override
    public Trigger leftBumper() {
        return button(Button.LeftBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightBumper() {
        return button(Button.RightBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftStick() {
        return button(Button.LeftStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightStick() {
        return button(Button.RightStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger a() {
        return button(Button.A.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger b() {
        return button(Button.B.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger x() {
        return button(Button.X.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger y() {
        return button(Button.Y.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger start() {
        return button(Button.Plus.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger back() {
        return button(Button.Minus.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftTrigger() {
        return leftTrigger(0.5);
    }

    @Override
    public Trigger leftTrigger(double threshold) {
        return axisGreaterThan(Axis.LeftTrigger.value, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightTrigger() {
        return rightTrigger(0.5);
    }

    @Override
    public Trigger rightTrigger(double threshold) {
        return axisGreaterThan(Axis.RightTrigger.value, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public double getLeftX() {
        return MathUtil.clamp(getRawAxis(Axis.LeftStickX.value) * STICK_MULTIPLIER, -1.0, 1.0);
    }

    @Override
    public double getLeftY() {
        return MathUtil.clamp(getRawAxis(Axis.LeftStickY.value) * STICK_MULTIPLIER, -1.0, 1.0);
    }

    @Override
    public double getRightX() {
        return MathUtil.clamp(getRawAxis(Axis.RightStickX.value) * STICK_MULTIPLIER, -1.0, 1.0);
    }

    @Override
    public double getRightY() {
        return MathUtil.clamp(getRawAxis(Axis.RightStickY.value) * STICK_MULTIPLIER, -1.0, 1.0);
    }

    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.clamp(getRawAxis(Axis.LeftTrigger.value), 0.0, 1.0);
    }

    @Override
    public double getRightTriggerAxis() {
        return MathUtil.clamp(getRawAxis(Axis.RightTrigger.value), 0.0, 1.0);
    }

    public Trigger povUp() {
        return button(Button.DpadUp.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger povUpRight() {
        return povUp().and(povRight());
    }

    public Trigger povRight() {
        return button(Button.DpadRight.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger povDownRight() {
        return povDown().and(povRight());
    }

    public Trigger povDown() {
        return button(Button.DpadDown.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger povDownLeft() {
        return povDown().and(povLeft());
    }

    public Trigger povLeft() {
        return button(Button.DpadLeft.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger povUpLeft() {
        return povUp().and(povLeft());
    }

    public Trigger povCenter() {
        return povUpLeft().and(povDownRight()).negate();
    }

    public enum Button {
        LeftBumper(5),
        RightBumper(6),
        LeftStick(7),
        RightStick(8),
        A(2),
        B(1),
        X(4),
        Y(3),
        Plus(9),
        Minus(10),
        DpadUp(12),
        DpadRight(13),
        DpadDown(14),
        DpadLeft(15);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        LeftStickX(0),
        LeftStickY(1),
        RightStickX(4),
        RightStickY(5),
        LeftTrigger(2),
        RightTrigger(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }
}
