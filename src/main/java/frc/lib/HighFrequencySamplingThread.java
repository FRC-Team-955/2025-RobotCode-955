package frc.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HighFrequencySamplingThread extends Thread {
    public static final double frequencyHz = 250.0;

    public static final Lock highFrequencyLock = new ReentrantLock();
    private static HighFrequencySamplingThread instance = null;
    private final Lock signalsLock = new ReentrantLock();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();

    private final List<DoubleSupplier> genericDoubleSignals = new ArrayList<>();
    private final List<Queue<Double>> genericDoubleQueues = new ArrayList<>();
    private final List<BooleanSupplier> genericBooleanSignals = new ArrayList<>();
    private final List<Queue<Boolean>> genericBooleanQueues = new ArrayList<>();

    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];

    private HighFrequencySamplingThread() {
        setName("HighFrequencySamplingThread");
        setDaemon(true);
        super.start();
    }

    public static HighFrequencySamplingThread get() {
        if (instance == null) {
            instance = new HighFrequencySamplingThread();
        }
        return instance;
    }

    @Override
    public synchronized void start() {}

    public Queue<Double> registerPhoenixSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerSparkSignal(SparkBase spark, DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            sparks.add(spark);
            sparkSignals.add(signal);
            sparkQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerGenericSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            genericDoubleSignals.add(signal);
            genericDoubleQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    public Queue<Boolean> registerGenericSignal(BooleanSupplier signal) {
        Queue<Boolean> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            genericBooleanSignals.add(signal);
            genericBooleanQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        highFrequencyLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            highFrequencyLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            signalsLock.lock();
            try {
                if (Constants.CANivore.isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / frequencyHz, phoenixSignals);
                } else {
                    Thread.sleep((long) (1000.0 / frequencyHz));
                    if (phoenixSignals.length > 0) {
                        BaseStatusSignal.refreshAll(phoenixSignals);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            highFrequencyLock.lock();
            try {
                double timestamp = Timer.getFPGATimestamp();
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < sparkSignals.size(); i++) {
                    double value = sparkSignals.get(i).getAsDouble();
                    sparkQueues.get(i).offer(value);
                }
                for (int i = 0; i < genericDoubleSignals.size(); i++) {
                    genericDoubleQueues.get(i).offer(genericDoubleSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < genericBooleanSignals.size(); i++) {
                    genericBooleanQueues.get(i).offer(genericBooleanSignals.get(i).getAsBoolean());
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            } finally {
                highFrequencyLock.unlock();
            }
        }
    }
}
