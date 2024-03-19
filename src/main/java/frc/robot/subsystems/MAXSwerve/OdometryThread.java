package frc.robot.subsystems.MAXSwerve;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class OdometryThread {
    
    private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
    private List<Queue<Double>> queues = new ArrayList<>();
    private List<Queue<Double>> timestampQueues = new ArrayList<>();

    private Notifier odometryThread;

    private static OdometryThread instance;

    public static OdometryThread getInstance(){
        if(instance == null){
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread(){
        odometryThread = new Notifier(this::periodic);
        odometryThread.setName("Odometry Thread");
    }

    public void start(){
        if(timestampQueues.size() > 0) {
            odometryThread.startPeriodic(1.0 / Constants.odometryFrequency);
        }
    }

    public Queue<Double> registerSignal(Supplier<OptionalDouble> signal){
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerTimestamps(){
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    private void periodic(){
        Drive.odometryLock.lock();
        double timeStamp = Logger.getRealTimestamp() / 1e6;
        try {
            double[] values = new double[signals.size()];
            boolean valid = true;
            for(int i = 0; i < signals.size(); i++){
                OptionalDouble value = signals.get(i).get();
                if(value.isPresent()){
                    values[i] = value.getAsDouble();
                } else {
                    valid = false;
                    break;
                }
            }
            if(valid){
                for(int i = 0; i < queues.size(); i++){
                    queues.get(i).offer(values[i]);
                }
                for(int i = 0; i < timestampQueues.size(); i++){
                    timestampQueues.get(i).offer(timeStamp);
                }
            }
        } finally {
            Drive.odometryLock.unlock();
        }
    }

}
