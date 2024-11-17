package org.firstinspires.ftc.teamcode.utility;

public class MotionProfile {
    public double jerk;
    public double cruiseVelocity;
    public double finalPosition;
    public double[] timeIntervals;


    public MotionProfile(double jerk1, double cruiseVelocity1, double finalPosition1) {
        jerk = jerk1;
        cruiseVelocity = cruiseVelocity1;
        finalPosition = finalPosition1;
        timeIntervals = computeTimeIntervals();
    }
    // Function to compute the time intervals based on final position, jerk, and cruise velocity
    public double[] computeTimeIntervals() {
        // First, calculate the time to accelerate to cruise velocity (t1)
        double t1 = Math.sqrt(2 * cruiseVelocity / jerk);

        // Time spent cruising (t2 - t1) can be derived by considering the final position
        double positionAtT1 = (jerk * Math.pow(t1, 3)) / 6;
        double cruiseTime = (finalPosition - positionAtT1) / cruiseVelocity;
        double t2 = t1 + cruiseTime;

        // Time spent decelerating (t3 - t2)
        double timeDeceleration = Math.sqrt(2 * cruiseVelocity / jerk);
        double t3 = t2 + timeDeceleration;

        // Return the time intervals [t1, t2, t3]
        return new double[] {t1, t2, t3};
    }

    // Function to compute position, velocity, and acceleration at a given time
    public double[] computeMotion(double time) {
        // Calculate time intervals based on final position
        double t1 = timeIntervals[0];
        double t2 = timeIntervals[1];
        double t3 = timeIntervals[2];

        double[] results = new double[3]; // Array to store [position, velocity, acceleration]

        if (time <= t1) {
            // Acceleration Phase (0 <= t <= t1)
            double acceleration = jerk * time; // a(t) = j0 * t
            double velocity = (jerk * Math.pow(time, 2)) / 2; // v(t) = (j0 * t^2) / 2
            double position = (jerk * Math.pow(time, 3)) / 6; // p(t) = (j0 * t^3) / 6
            results[0] = position;  // Position
            results[1] = velocity;  // Velocity
            results[2] = acceleration; // Acceleration
        } else if (time <= t2) {
            // Cruise Phase (t1 < t <= t2)
            double positionAtT1 = (jerk * Math.pow(t1, 3)) / 6;
            double velocityAtT1 = (jerk * Math.pow(t1, 2)) / 2;
            double position = positionAtT1 + cruiseVelocity * (time - t1); // p(t) = p(t1) + v_cruise * (t - t1)
            results[0] = position; // Position
            results[1] = cruiseVelocity; // Velocity
            results[2] = 0; // Acceleration is 0 during cruise
        } else if (time <= t3) {
            // Deceleration Phase (t2 < t <= t3)
            double positionAtT2 = (jerk * Math.pow(t1, 3)) / 6 + cruiseVelocity * (t2 - t1);
            double velocityAtT2 = cruiseVelocity;
            double timeDeceleration = time - t2;
            double position = positionAtT2 + cruiseVelocity * timeDeceleration - (jerk * Math.pow(timeDeceleration, 3)) / 6;
            double velocity = cruiseVelocity - (jerk * Math.pow(timeDeceleration, 2)) / 2; // v(t) = v_cruise - (j0 * (t - t2)^2) / 2
            double acceleration = -jerk * timeDeceleration; // a(t) = -j0 * (t - t2)
            results[0] = position;  // Position
            results[1] = velocity;  // Velocity
            results[2] = acceleration; // Acceleration
        } else {
            // After the deceleration phase ends (position has reached final value)
            results[0] = finalPosition;  // Final position
            results[1] = 0;   // Final velocity
            results[2] = 0;   // Final acceleration
        }

        return results; // returns [position, velocity, acceleration]
    }

    public void main(String[] args) {
//        jerk = 1; // Jerk (rate of change of acceleration)
//        cruiseVelocity = 10; // Maximum cruising velocity
//        finalPosition = 12; // Final position (input by user)

//        time = 6; // Time at which we want to compute position, velocity, and acceleration

//        double[] result = computeMotion(time, finalPosition, jerk, cruiseVelocity);
        // result[], [0] = current pos, [1] = current velo, [2] = current accel
//        System.out.println("At time " + time + " seconds:");
//        System.out.println("Position: " + result[0]);
//        System.out.println("Velocity: " + result[1]);
//        System.out.println("Acceleration: " + result[2]);
    }
}


