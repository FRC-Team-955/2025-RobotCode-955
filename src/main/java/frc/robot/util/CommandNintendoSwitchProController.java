package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandNintendoSwitchProController extends CommandXboxController {
    private static final double STICK_MULTIPLIER = 1 / 0.75; // Sticks normally only go to 0.75 max

    // https://squircular.blogspot.com/2015/09/mapping-circle-to-square.html
    private static double circleToSquareX(double u, double v) {
        double u2 = u * u;
        double v2 = v * v;
        double twosqrt2 = 2.0 * Math.sqrt(2.0);
        double subtermx = 2.0 + u2 - v2;
        double termx1 = subtermx + u * twosqrt2;
        double termx2 = subtermx - u * twosqrt2;
        return 0.5 * Math.sqrt(termx1) - 0.5 * Math.sqrt(termx2);
    }

    private static double circleToSquareY(double u, double v) {
        double u2 = u * u;
        double v2 = v * v;
        double twosqrt2 = 2.0 * Math.sqrt(2.0);
        double subtermy = 2.0 - u2 + v2;
        double termy1 = subtermy + v * twosqrt2;
        double termy2 = subtermy - v * twosqrt2;
        return 0.5 * Math.sqrt(termy1) - 0.5 * Math.sqrt(termy2);
    }

    public CommandNintendoSwitchProController(int port) {
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
        return button(Button.LeftTrigger.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftTrigger(double threshold) {
        return leftTrigger();
    }

    @Override
    public Trigger rightTrigger() {
        return button(Button.RightTrigger.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightTrigger(double threshold) {
        return rightTrigger();
    }

    private double getLeftXRaw() {
        return getRawAxis(0) * STICK_MULTIPLIER;
    }

    private double getLeftYRaw() {
        return getRawAxis(1) * STICK_MULTIPLIER;
    }

    @Override
    public double getLeftX() {
        return MathUtil.clamp(circleToSquareX(getLeftXRaw(), getLeftYRaw()), -1.0, 1.0);
    }

    @Override
    public double getLeftY() {
        return MathUtil.clamp(circleToSquareY(getLeftXRaw(), getLeftYRaw()), -1.0, 1.0);
    }

    private double getRightXRaw() {
        return getRawAxis(2) * STICK_MULTIPLIER;
    }

    private double getRightYRaw() {
        return getRawAxis(3) * STICK_MULTIPLIER;
    }

    @Override
    public double getRightX() {
        return MathUtil.clamp(circleToSquareX(getRightXRaw(), getRightYRaw()), -1.0, 1.0);
    }

    @Override
    public double getRightY() {
        return MathUtil.clamp(circleToSquareY(getRightXRaw(), getRightYRaw()), -1.0, 1.0);
    }

    @Override
    public double getLeftTriggerAxis() {
        return getHID().getRawButton(Button.LeftTrigger.value) ? 1 : 0;
    }

    @Override
    public double getRightTriggerAxis() {
        return getHID().getRawButton(Button.RightTrigger.value) ? 1 : 0;
    }

    public enum Button {
        LeftBumper(5),
        RightBumper(6),
        LeftStick(11),
        RightStick(12),
        // note: A is really 2, but having it as 1 matches the button position with an xbox controller
        A(1),
        // note: B is really 1, but having it as 2 matches the button position with an xbox controller
        B(2),
        // note: X is really 4, but having it as 1 matches the button position with an xbox controller
        X(3),
        // note: Y is really 3, but having it as 1 matches the button position with an xbox controller
        Y(4),
        Plus(10),
        Minus(9),
        LeftTrigger(7),
        RightTrigger(8);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }
}
