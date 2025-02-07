package frc.robot.util.characterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** kG characterization for a static mechanism (e.g. an elevator) */
public class StaticGravityCharacterization extends Command {
    private static final double startDelaySec = 0;
    private static final double rampVoltsPerSec = 0.01;
    private static final double initialVoltage = 5.75;

    private Data data;
    private final Consumer<Double> voltageConsumer;
    private final Supplier<Double> velocitySupplier;

    private final Timer timer = new Timer();

    public StaticGravityCharacterization(Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier, Subsystem subsystem) {
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        data = new Data();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() < startDelaySec) {
            voltageConsumer.accept(0.0);
        } else {
            double voltage = initialVoltage + (timer.get() - startDelaySec) * rampVoltsPerSec;
            voltageConsumer.accept(voltage);
            data.add(velocitySupplier.get(), voltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(0.0);
        timer.stop();
        data.printResults();
    }

    private static class Data {
        private final List<DataPoint> data = new LinkedList<>();

        public void add(double velocity, double voltage) {
            data.add(new DataPoint(velocity, voltage));
        }

        public void printResults() {
            var closestPositiveOptional = data.stream()
                    .filter(d -> d.velocity >= 0)
                    .min(Comparator.comparingDouble(DataPoint::velocity));

            var closestNegativeOptional = data.stream()
                    .filter(d -> d.velocity < 0)
                    .max(Comparator.comparingDouble(DataPoint::velocity));

            if (closestPositiveOptional.isEmpty() || closestNegativeOptional.isEmpty()) {
                return;
            }

            var closestPositive = closestPositiveOptional.get();
            var closestNegative = closestNegativeOptional.get();

            System.out.println(closestPositive);
            System.out.println(closestNegative);

            // Interpolate linearly
            var voltageAtZero = (closestNegative.voltage * closestPositive.velocity + closestPositive.voltage * -closestNegative.velocity)
                    / (closestPositive.velocity - closestNegative.velocity);

            System.out.println("Static Gravity Characterization Results:");
            System.out.println("\tCount=" + data.size());
            System.out.printf("\tkG=%.5f%n", voltageAtZero);
        }
    }

    private record DataPoint(double velocity, double voltage) {}
}
