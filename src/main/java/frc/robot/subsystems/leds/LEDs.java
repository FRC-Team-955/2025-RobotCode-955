package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.subsystem.SubsystemBaseExt;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.leds.LEDConstants.*;
import static frc.robot.subsystems.leds.LEDPatterns.*;

public class LEDs extends SubsystemBaseExt {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Superstructure superstructure = Superstructure.get();

    private final LEDsIO io = createIO();

    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

    /** Relative to front of robot; ReefCam side. 0 = top */
    private final AddressableLEDBufferView leftView = new AddressableLEDBufferView(buffer, 0, length / 2 - 1);
    private final AddressableLEDBufferView leftUpperView = new AddressableLEDBufferView(leftView, 0, length / 4 - 1);
    private final AddressableLEDBufferView leftLowerView = new AddressableLEDBufferView(leftView, length / 4, length / 2 - 1);

    /** Relative to front of robot; StationCam side. 0 = top */
    private final AddressableLEDBufferView rightView = new AddressableLEDBufferView(buffer, length / 2, length - 1).reversed();
    private final AddressableLEDBufferView rightUpperView = new AddressableLEDBufferView(rightView, 0, length / 4 - 1);
    private final AddressableLEDBufferView rightLowerView = new AddressableLEDBufferView(rightView, length / 4, length / 2 - 1);

    public boolean autonomousRunning = false;
    private boolean lowBattery = false;
    private boolean endgame = false;

    private static LEDs instance;

    public static LEDs get() {
        if (instance == null)
            synchronized (LEDs.class) {
                instance = new LEDs();
            }

        return instance;
    }

    private LEDs() {
        new Trigger(() -> RobotController.getBatteryVoltage() <= lowBatteryThresholdVolts)
                .debounce(10.0)
                .onTrue(Commands.runOnce(() -> lowBattery = true))
                .onFalse(Commands.runOnce(() -> lowBattery = false));

        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < endgameThresholdSeconds)
                .onTrue(Commands.startEnd(() -> endgame = true, () -> endgame = false).withTimeout(5.0));
    }

    public Notifier createAndStartStartupNotifier() {
        LEDPattern pattern = startup();
        Notifier notifier = new Notifier(() -> {
            pattern.applyTo(buffer);
            io.setData(buffer);
        });
        notifier.startPeriodic(0.02);
        return notifier;
    }

    private void set(LEDPattern pattern) {
        pattern.applyTo(leftView);
        pattern.applyTo(rightView);
    }

    private void set(LEDPattern upper, LEDPattern lower) {
        upper.applyTo(leftUpperView);
        upper.applyTo(rightUpperView);
        lower.applyTo(leftLowerView);
        lower.applyTo(rightLowerView);
    }

    @Override
    public void periodicAfterCommands() {
        boolean overrideSet = operatorDashboard.coastOverride.get()
                || operatorDashboard.ignoreEndEffectorBeamBreak.get();

        if (viewDebug) {
            LEDPattern.solid(Color.kRed).blink(Seconds.of(1)).applyTo(leftUpperView);
            LEDPattern.solid(Color.kGreen).blink(Seconds.of(1)).applyTo(leftLowerView);
            LEDPattern.solid(Color.kYellow).blink(Seconds.of(1)).applyTo(rightUpperView);
            LEDPattern.solid(Color.kBlue).blink(Seconds.of(1)).applyTo(rightLowerView);
        } else if (DriverStation.isAutonomousEnabled()) {
            if (autonomousRunning) {
                set(patternForAlliance(autoRed, autoBlue, autoUnknown));
            } else {
                set(autoFinished);
            }
        } else if (DriverStation.isTeleopEnabled()) {
            LEDPattern pattern = switch (superstructure.getGoal()) {
                case AUTO_SCORE_CORAL_WAIT_INITIAL, AUTO_SCORE_CORAL_SCORING,
                     AUTO_FUNNEL_INTAKE_WAITING_ALIGN, AUTO_FUNNEL_INTAKE_WAITING_SHAKE -> autoScoring;

                case AUTO_SCORE_CORAL_WAIT_FINAL, AUTO_SCORE_CORAL_WAIT_ELEVATOR ->
                        superstructure.isAutoScoreForceable() ? driverConfirm : autoScoring;

                case DESCORE_ALGAE_WAIT_ELEVATOR, MANUAL_SCORE_CORAL_WAIT_ELEVATOR -> waitElevator;

                case FUNNEL_INTAKE_WAITING -> funnelIntaking;

                case MANUAL_SCORE_CORAL_WAIT_CONFIRM -> driverConfirm;

                case FUNNEL_INTAKE_FINALIZING, AUTO_FUNNEL_INTAKE_FINALIZING,
                     MANUAL_SCORE_CORAL_SCORING, DESCORE_ALGAE_DESCORING -> finalizing;

                case EJECT -> eject;

                case IDLE -> {
                    if (superstructure.endEffectorTriggeredLong()) {
                        yield endEffectorTriggered;
                    } else {
                        yield idle;
                    }
                }
            };
            if (endgame) {
                set(pattern, endgameAlert);
            } else if (lowBattery) {
                set(pattern, lowBatteryAlert);
            } else if (constantSet) {
                set(pattern, constantSetAlert);
            } else if (overrideSet) {
                set(pattern, overrideAlert);
            } else {
                set(pattern);
            }
        } else {
            LEDPattern pattern = overrideSet
                    ? overrideAlert
                    : patternForAlliance(disabled, disabledUnknown);
            if (lowBattery) {
                set(pattern, lowBatteryAlert);
            } else if (constantSet) {
                set(pattern, constantSetAlert);
            } else if (!operatorDashboard.autoChosen.get()) {
                set(pattern, autoNotChosen);
            } else {
                set(pattern);
            }
        }

        io.setData(buffer);
    }
}