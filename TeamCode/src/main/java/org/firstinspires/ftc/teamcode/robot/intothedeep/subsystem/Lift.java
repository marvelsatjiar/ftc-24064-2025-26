package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.util.CachedMotor;

@Config
public final class Lift {

    /**
     * A PIDGains object being set to certain values (tweak these numbers!!)
     */
    public static PIDGains pidGains = new PIDGains(
            0.006,
            0.00004,
            0,
            Double.POSITIVE_INFINITY
    );

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.01,
            0.01
    );

    // Sets the filter for PID outputs and constrains overshoots with controlling (also tweak!!)
    public static LowPassGains filterGains = new LowPassGains(0, 2);

    /**
     * Sets the constants for the positions, conversions, etc
     * Remember to set these constants correctly! (in ticks)
     */
    public static int
            MAX_MOTOR_TICKS = 2550,
            MIN_MOTOR_TICKS = -5,
            LOW_BASKET_TICKS = 297,
            HIGH_BASKET_TICKS = 1670,
            WALL_PICKUP_TICKS = 953,
            BEFORE_BACK_SPECIMEN_TICKS = 1043,
            SPECIMEN_SETUP_TICKS = 1438,
            AUTON_FIRST_SPECIMEN_TICKS = 1470,
            INTERLEAVE_TICKS = 150,
            HANG_SETUP_TICKS = 2550,
            HANG_TICKS = 1400,
            UNSAFE_THRESHOLD_TICKS = 829,
            RETRACTED_THRESHOLD_TICKS = 41,
            CHAMBER_HANG_OVERRIDE_TICKS = 50;

    public static double
            kG = 0.011065,
            RESETTING_POWER = -0.02,
            JOYSTICK_MULTIPLIER = 40; // 1 = 40 ticks

    private final MotorEx[] motors;

    private final Motor.Encoder encoder;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private double position = 0;

    public enum Ticks {
        RETRACTED,
        LOW_BASKET,
        HIGH_BASKET,
        WALL_PICKUP,
        BEFORE_BACK_SPECIMEN,
        SETUP_SPECIMEN,
        AUTON_SETUP_FIRST_SPECIMEN,
        HANG_SETUP,
        INTERLEAVE,
        HANG,
        CHAMBER_HANG_OVERRIDE,
        EXTENDED;

        private int toTicks() {
            switch (this) {
                case LOW_BASKET:                return LOW_BASKET_TICKS;
                case HIGH_BASKET:               return HIGH_BASKET_TICKS;
                case WALL_PICKUP:               return WALL_PICKUP_TICKS;
                case BEFORE_BACK_SPECIMEN:      return BEFORE_BACK_SPECIMEN_TICKS;
                case SETUP_SPECIMEN:            return SPECIMEN_SETUP_TICKS;
                case AUTON_SETUP_FIRST_SPECIMEN: return AUTON_FIRST_SPECIMEN_TICKS;
                case INTERLEAVE:                return INTERLEAVE_TICKS;
                case HANG:                      return HANG_TICKS;
                case HANG_SETUP:                return HANG_SETUP_TICKS;
                case EXTENDED:                  return MAX_MOTOR_TICKS;
                case CHAMBER_HANG_OVERRIDE:     return CHAMBER_HANG_OVERRIDE_TICKS;
                case RETRACTED: default:        return MIN_MOTOR_TICKS;
            }
        }

        public boolean isArmUnsafe() {
            return toTicks() <= UNSAFE_THRESHOLD_TICKS;
        }
    }

    private Ticks targetTicks = Ticks.RETRACTED;

    public boolean isLocked = false;

    private double manualPower;
    private double finalOutput;

    /**
     * Constructor of Lift class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Lift(HardwareMap hardwareMap) {
        CachedMotor leader = new CachedMotor(hardwareMap, "leader", RPM_435);
        CachedMotor follower = new CachedMotor(hardwareMap, "follower", RPM_435);

        MotorEx dummy = new MotorEx(hardwareMap, "follower2", RPM_435);

        encoder = dummy.encoder;
        encoder.reset();

        follower.setInverted(true);
        dummy.setInverted(true);

        encoder.setDirection(Motor.Direction.REVERSE);

        motors = new MotorEx[] {leader, follower};

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public Ticks getTargetTicks() {
        return targetTicks;
    }

    public boolean setTargetTicks(Ticks ticks, boolean isOverride ) {
        if (isLocked && !isOverride) return false;
        targetTicks = ticks;
        controller.setTarget(new State(getTargetTicks().toTicks()));

        return true;
    }

    public void runManual(double power) {
        manualPower = power;
    }

    public void reset() {
        encoder.reset();
        controller.reset();

        setTargetTicks(Ticks.RETRACTED, true);
        position = 0;
    }

    public boolean setTargetTicks(Ticks ticks) {
        return setTargetTicks(ticks, false);
    }

    /**
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Calls another run() method that calculates the motor output proportionally and doesn't compensate for power
     */
    public void run() {
        position = encoder.getPosition();

        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = position >= RETRACTED_THRESHOLD_TICKS ? kG * scalar : 0;


         if (manualPower != 0) {
            controller.setTarget(new State(position));
            output += manualPower;

        } else if (targetTicks == Ticks.RETRACTED && position <= RETRACTED_THRESHOLD_TICKS) {
            output = RESETTING_POWER;
            this.reset();

        } else {

            controller.setGains(pidGains);
            derivFilter.setGains(filterGains);

            output += controller.calculate(new State(position));

        }


        for (MotorEx motor : motors) motor.set(output);
        finalOutput = output;
    }

    public void printTelemetry() {
        mTelemetry.addData("Target position (ticks)", getTargetTicks().toTicks());
        mTelemetry.addData("Current position (ticks)", position);
        mTelemetry.addData("Current state (name)", getTargetTicks().name());
        mTelemetry.addData("Manual power (%)", manualPower * 100);
        mTelemetry.addData("Motor power (%)", finalOutput * 100);
    }
}
