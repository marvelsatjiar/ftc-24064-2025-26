package org.firstinspires.ftc.teamcode.control.filter.dualfilter;

public interface DualFilter {
    default double calculate(double value1, double value2) {
        return Double.NaN;
    }

    default void reset() {}
}
