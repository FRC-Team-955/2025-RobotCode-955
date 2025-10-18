package frc.lib.characterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FeedforwardCharacterization extends Command {
    private static final double START_DELAY_SECS = 2.0;
    private static final double RAMP_VOLTS_PER_SEC = 0.25;

    private FeedforwardCharacterizationData[] data;
    private final int numberOfVelocity;
    private final Consumer<Double> voltageConsumer;
    private final Supplier<double[]> velocitySupplier;

    private final Timer timer = new Timer();

    public FeedforwardCharacterization(
            Consumer<Double> voltageConsumer,
            Supplier<double[]> velocitySupplier,
            int numberOfVelocity,
            Subsystem subsystem
    ) {
       this.voltageConsumer = voltageConsumer;
       this.numberOfVelocity = numberOfVelocity;
       this.velocitySupplier = velocitySupplier;
       addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        data = new FeedforwardCharacterizationData[numberOfVelocity];
        for (int i = 0; i < numberOfVelocity; i++) {
            data[i] = new FeedforwardCharacterization();
        }
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() < START_DELAY_SECS) {
            voltageConsumer.accept(0.0);
        } else {
            double voltage = (timer.get() - START_DELAY_SECS) *RAMP_VOLTS_PER_SEC;
            voltageConsumer.accept(voltage);
            var velocities = velocitySupplier.get();
            for (int i = 0; i < numberOfVelocity; i++) {
                data[i].add(velocities[i], voltage);
            }
        }
    }
    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(0.0);
        timer.stop();
        for(int i = 0; i < numberOfVelocity; i++){
            System.out.println("Velocity #" + i);
            data[i].printResults();
        }
    }

    private static class FeedforwardCharacterizationData {
        private final List<Double> velocityData = new LinkedList<>();
        private final List<Double> voltageData = new LinkedList<>();

        public void add(double velocity, double voltage) {
            if (Math.abs(velocity) > 1E-4) {
                velocityData.add(Math.abs(velocity));
                voltageData.add(Math.abs(voltage));
            }
        }

        public void printResults() {
            if (velocityData.isEmpty() || voltageData.isEmpty()){
                return;
            }

            PolynomialRegression regression = new PolynomialRegression(
                    velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                    voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
                    1
            );

            System.out.println("Feedforward Characterization Results:");
            System.out.println("\tCount=" +velocityData.size());
            System.out.printf("\tR2=%.5f%n", regression.R2());
            System.out.printf("\tkS=%.5f%n", regression.beta(0));
            System.out.printf("\tkC=%.5f%n", regression.beta(1));
        }
    }
}
