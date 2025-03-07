package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

public class TimestampAverageBuffer {
    List<Pair<Double, Double>> valueTimestampList = new ArrayList<>();
    private final double periodSeconds;

    public TimestampAverageBuffer(double periodSeconds) {
        this.periodSeconds = periodSeconds;
    }

    public void addValue(double value, double timestamp) {
        valueTimestampList.add(new Pair<>(value, timestamp));
    }

    // gets average across period
    public double getAverage() {
         valueTimestampList = valueTimestampList.stream().filter((pair) -> Timer.getFPGATimestamp() - pair.getSecond() < periodSeconds).toList();
         double avg = 0;
         for (Pair<Double, Double> pair : valueTimestampList) {
             avg += pair.getFirst();
         }

         avg /= valueTimestampList.size();
         return avg;
    }
}
