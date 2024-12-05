/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autoModes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.MotionProfile;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Specimin", group="Robot")
public class Specimin extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;


    public DcMotorEx armMotor = null;
    public DcMotorEx elbowMotor = null;

    public Servo claw = null;
    public Servo wrist = null;
    public CRServo intake = null;


    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 480 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public MotionProfile trapezoid;
    static final double pivotJerk = 3000;


    private final double ticks_in_degree = 1120 / 270;
    private PIDController controller;
    public static double p = 0.01, i = 0.05, d = 0;
    public static double f = 0.6;

    private PIDController controller2;
    public static double p2 = .015, i2 = 0.075, d2 = 0;
    public static double f2 = 0;

    private PIDController controller3;
    public static double p3 = 0.01, i3 = 0.04, d3 = 0;
    public static double f3 = 0.5;

    public static double armTarget;
    public static double elbowTarget;




    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        controller3 = new PIDController(p3, i3, d3);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Initialize the drive system variables.

        frontLeft = hardwareMap.get(DcMotorEx.class,"leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class,"rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class,"leftRear");
        backRight  = hardwareMap.get(DcMotorEx.class,"rightRear");


        intake = hardwareMap.get(CRServo.class, "Intake");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");

        armMotor = hardwareMap.get(DcMotorEx.class, "armPivot");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");

        armTarget = armMotor.getCurrentPosition();
        elbowTarget = armMotor.getCurrentPosition();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // commands in main loop
        runtime.reset();
//        encoderDrive(DRIVE_SPEED,  24,  24, 5.0, imu);  // S1: Forward 47 Inches with 5 Sec timeout

        clawset(true);
        armPositionSet(-1250,700);
        //score
        drive(31,31,0.15); //was 20 and 20
        clawset(false);
        armPositionSet(-850,700);

        //move back
        drive(-22,-22,0.1);
        armPositionSet(0,0);
        //move towards blocks
        strafe(-44,0.05);
        drive(52,52, 0.1); //changed with updated angles
        strafe(-18, 0.1);
        // push first block
        drive(-46,-46,0.1);
        drive(50,50,0.1);
        drive(2,-2,0.1);
        strafe(-12,0.1);
        // second block
        drive(-50,-50,0.1);
        drive(50,50,0.1);
        drive(3,-3,0.1);
        strafe(-12,0.1);

        // third block
        drive(-52,-52,0.15);

//        encoderDrive(TURN_SPEED,   12, -12, 4.0, imu);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0, imu);  // S3: Reverse 24 Inches with 4 Sec timeout


        sleep(10000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, BNO055IMU imu) {

//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        double botHeading = 0;
        leftInches = ((leftInches/3)/11)*12;
        rightInches = ((rightInches/3)/11)*12;

        double botHeading = imu.getAngularOrientation().firstAngle;
        if (botHeading > (2 * Math.PI)){
            botHeading -= (2*Math.PI);
        } else if (botHeading < 0){
            botHeading += (2*Math.PI);
        }
        double x = 0;
        double y = 0;
        double turn = 0;

        // Rotate gamepad input x/y by hand
        // See: https://matthew-brett.github.io/teaching/rotation_2d.html
        double rotX = (x * Math.sin(botHeading)) + (y * Math.cos(botHeading));
        double rotY = (y * Math.sin(botHeading)) - (x * Math.cos(botHeading));


        double frontLeftPower = -rotX - rotY + turn;
        double backLeftPower = -rotX + rotY - turn;
        double frontRightPower = -rotX + rotY + turn;
        double backRightPower = -rotX - rotY - turn;



        // Put powers in the range of -1 to 1 only if they aren't already
        // Not checking would cause us to always drive at full speed
        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);



            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }



        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newBackRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // jerk, cruise velocity, target
            // inches is of course 12in = 1ft
            // the motor needs to go 114 counts per inch
            // thus the cruise velocity will need to be near 6in per sec

            // reset the timeout time and start motion


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (frontLeft.isBusy() || frontRight.isBusy() || backRight.isBusy() || backLeft.isBusy())) {
                double pid = controller.calculate(frontRight.getCurrentPosition(), newFrontRightTarget);
                double ff = Math.cos(Math.toRadians(newFrontRightTarget / ticks_in_degree)) * f;
                double power = pid + ff;
                frontRight.setPower(power);

                double pid1 = controller.calculate(frontLeft.getCurrentPosition(), newFrontLeftTarget);
                double ff1 = Math.cos(Math.toRadians(newFrontLeftTarget / ticks_in_degree)) * f;
                double power1 = pid1 + ff1;
                frontRight.setPower(power1);

                double pid2 = controller.calculate(backRight.getCurrentPosition(), newBackRightTarget);
                double ff2 = Math.cos(Math.toRadians(newBackRightTarget / ticks_in_degree)) * f;
                double power2 = pid2 + ff2;
                frontRight.setPower(power2);

                double pid3 = controller.calculate(backLeft.getCurrentPosition(), newBackLeftTarget);
                double ff3 = Math.cos(Math.toRadians(newBackLeftTarget / ticks_in_degree)) * f;
                double power3 = pid3 + ff3;
                frontRight.setPower(power3);


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION

            sleep(500);   // optional pause after each move.
        }


    }
    public void armPositionSet(int armTarget1, int elbowTarget1) {
        armTarget = armTarget1;
        elbowTarget = elbowTarget1;
        runtime.reset();
        int rangeVariable = 15;
        while (opModeIsActive() && (runtime.seconds() < 1.5) && !(
                ((armMotor.getCurrentPosition() >= armTarget-rangeVariable) && (rangeVariable+armTarget >= armMotor.getCurrentPosition())) &&
                ((elbowMotor.getCurrentPosition() >= elbowTarget-rangeVariable) && (rangeVariable+elbowTarget >= elbowMotor.getCurrentPosition()))
        )
        ) {
            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid4 = controller.calculate(armPos, armTarget);
            double ff4 = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
            double power4 = pid4 + ff4;
            armMotor.setPower(power4);

            controller2.setPID(p2, i2, d2);
            int elbowPos = elbowMotor.getCurrentPosition();
            double pid5 = controller2.calculate(elbowPos, elbowTarget);
            double ff5 = Math.cos(Math.toRadians(elbowTarget / ticks_in_degree)) * f2;
            double power5 = pid5 + ff5;
            elbowMotor.setPower(power5);

            telemetry.addData("armpower", armMotor.getPower());
            telemetry.addData("elbow", elbowMotor.getPower());
            telemetry.update();

        }

    }
    public void drive(double leftin, double rightin, double speedMultiplyer) {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double newFrontLeftTarget = frontLeft.getCurrentPosition() + (leftin * COUNTS_PER_INCH);
        double newFrontRightTarget = frontRight.getCurrentPosition() + (rightin * COUNTS_PER_INCH);
        double newBackLeftTarget = backLeft.getCurrentPosition() + (leftin * COUNTS_PER_INCH);
        double newBackRightTarget = frontRight.getCurrentPosition() + (rightin * COUNTS_PER_INCH);

        int rangeVariable = 15;
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < Math.sqrt(Math.abs(leftin))/2.75 ) && !( //changed from 2.75
                ((frontRight.getCurrentPosition() >= newFrontRightTarget-rangeVariable) && (rangeVariable+newFrontRightTarget >= frontRight.getCurrentPosition())) &&
                ((frontLeft.getCurrentPosition() >= newFrontLeftTarget-rangeVariable) && (rangeVariable+newFrontLeftTarget >= frontLeft.getCurrentPosition())) &&
                ((backLeft.getCurrentPosition() >= newBackLeftTarget-rangeVariable) && (rangeVariable+newBackLeftTarget >= backLeft.getCurrentPosition())) &&
                ((backRight.getCurrentPosition() >= newBackRightTarget-rangeVariable) && (rangeVariable+newBackRightTarget >= backRight.getCurrentPosition()))
                ) ) {

            double pid = controller3.calculate(frontRight.getCurrentPosition(), newFrontRightTarget);
            double ff = Math.cos(Math.toRadians(newFrontRightTarget / ticks_in_degree)) * f3;
            double power = pid + ff;
            frontRight.setPower(power*speedMultiplyer);

            double pid1 = controller3.calculate(frontLeft.getCurrentPosition(), newFrontLeftTarget);
            double ff1 = Math.cos(Math.toRadians(newFrontLeftTarget / ticks_in_degree)) * f3;
            double power1 = pid1 + ff1;
            frontLeft.setPower(power1*speedMultiplyer);

            double pid2 = controller3.calculate(backRight.getCurrentPosition(), newBackRightTarget);
            double ff2 = Math.cos(Math.toRadians(newBackRightTarget / ticks_in_degree)) * f3;
            double power2 = pid2 + ff2;
            backRight.setPower(power2*speedMultiplyer);

            double pid3 = controller3.calculate(backLeft.getCurrentPosition(), newBackLeftTarget);
            double ff3 = Math.cos(Math.toRadians(newBackLeftTarget / ticks_in_degree)) * f3;
            double power3 = pid3 + ff3;
            backLeft.setPower(power3*speedMultiplyer);

            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid4 = controller.calculate(armPos, armTarget);
            double ff4 = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
            double power4 = pid4 + ff4;
            armMotor.setPower(power4);

            controller2.setPID(p2, i2, d2);
            int elbowPos = elbowMotor.getCurrentPosition();
            double pid5 = controller2.calculate(elbowPos, elbowTarget);
            double ff5 = Math.cos(Math.toRadians(elbowTarget / ticks_in_degree)) * f2;
            double power5 = pid5 + ff5;
            elbowMotor.setPower(power5);

            telemetry.addData("armpower", armMotor.getPower());
            telemetry.addData("elbow", elbowMotor.getPower());
            telemetry.update();

            telemetry.addData("front left pos", newFrontLeftTarget);
            telemetry.addData("front right pos", newFrontRightTarget);
            telemetry.addData("back left pos", newBackLeftTarget);
            telemetry.addData("back right pos", newBackRightTarget);

            telemetry.addData("current front left pos", frontLeft.getCurrentPosition());
            telemetry.addData("current front right pos", frontRight.getCurrentPosition());
            telemetry.addData("current back left pos", backLeft.getCurrentPosition());
//            telemetry.addData("current back right pos", backRight.getCurrentPosition());

            telemetry.addData("armpower", armMotor.getPower());
            telemetry.addData("elbow", elbowMotor.getPower());

            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void strafe(double distance, double speedMultiplyer) {
        // left is positive
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double newFrontLeftTarget = frontLeft.getCurrentPosition() + (-distance * COUNTS_PER_INCH);
        double newFrontRightTarget = frontRight.getCurrentPosition() + (distance * COUNTS_PER_INCH);
        double newBackLeftTarget = backLeft.getCurrentPosition() + (distance * COUNTS_PER_INCH);
        double newBackRightTarget = backRight.getCurrentPosition() + (-distance * COUNTS_PER_INCH);

        int rangeVariable = 15;
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < Math.sqrt(Math.abs(distance))/3.5 ) && !( //tested to decrease time
                ((frontRight.getCurrentPosition() >= newFrontRightTarget - rangeVariable) && (rangeVariable + newFrontRightTarget >= frontRight.getCurrentPosition())) &&
                        ((frontLeft.getCurrentPosition() >= newFrontLeftTarget - rangeVariable) && (rangeVariable + newFrontLeftTarget >= frontLeft.getCurrentPosition())) &&
                        ((backLeft.getCurrentPosition() >= newBackLeftTarget - rangeVariable) && (rangeVariable + newBackLeftTarget >= backLeft.getCurrentPosition())) &&
                        ((backRight.getCurrentPosition() >= newBackRightTarget - rangeVariable) && (rangeVariable + newBackRightTarget >= backRight.getCurrentPosition()))
        )) {

            double pid = controller3.calculate(frontRight.getCurrentPosition(), newFrontRightTarget);
            double ff = Math.cos(Math.toRadians(newFrontRightTarget / ticks_in_degree)) * f3;
            double power = pid + ff;
            frontRight.setPower(power * speedMultiplyer);

            double pid1 = controller3.calculate(frontLeft.getCurrentPosition(), newFrontLeftTarget);
            double ff1 = Math.cos(Math.toRadians(newFrontLeftTarget / ticks_in_degree)) * f3;
            double power1 = pid1 + ff1;
            frontLeft.setPower(power1 * speedMultiplyer);

            double pid2 = controller3.calculate(backRight.getCurrentPosition(), newBackRightTarget);
            double ff2 = Math.cos(Math.toRadians(newBackRightTarget / ticks_in_degree)) * f3;
            double power2 = pid2 + ff2;
            backRight.setPower(power2 * speedMultiplyer);

            double pid3 = controller3.calculate(backLeft.getCurrentPosition(), newBackLeftTarget);
            double ff3 = Math.cos(Math.toRadians(newBackLeftTarget / ticks_in_degree)) * f3;
            double power3 = pid3 + ff3;
            backLeft.setPower(power3 * speedMultiplyer);

            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid4 = controller.calculate(armPos, armTarget);
            double ff4 = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
            double power4 = pid4 + ff4;
            armMotor.setPower(power4);

            controller2.setPID(p2, i2, d2);
            int elbowPos = elbowMotor.getCurrentPosition();
            double pid5 = controller2.calculate(elbowPos, elbowTarget);
            double ff5 = Math.cos(Math.toRadians(elbowTarget / ticks_in_degree)) * f2;
            double power5 = pid5 + ff5;
            elbowMotor.setPower(power5);

            telemetry.addData("front left pos", newFrontLeftTarget);
            telemetry.addData("front right pos", newFrontRightTarget);
            telemetry.addData("back left pos", newBackLeftTarget);
            telemetry.addData("back right pos", newBackRightTarget);

            telemetry.addData("current front left pos", frontLeft.getCurrentPosition());
            telemetry.addData("current front right pos", frontRight.getCurrentPosition());
            telemetry.addData("current back left pos", backLeft.getCurrentPosition());
            telemetry.addData("current back right pos", backRight.getCurrentPosition());

            telemetry.addData("armpower", armMotor.getPower());
            telemetry.addData("elbow", elbowMotor.getPower());

            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void clawset(boolean clawbol) {
        // true is close, false is open
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid4 = controller.calculate(armPos, armTarget);
            double ff4 = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
            double power4 = pid4 + ff4;
            armMotor.setPower(power4);

            controller2.setPID(p2, i2, d2);
            int elbowPos = elbowMotor.getCurrentPosition();
            double pid5 = controller2.calculate(elbowPos, elbowTarget);
            double ff5 = Math.cos(Math.toRadians(elbowTarget / ticks_in_degree)) * f2;
            double power5 = pid5 + ff5;
            elbowMotor.setPower(power5);

            if (clawbol == true) {
                claw.setPosition(0.95);
            } else if (clawbol == false) {
                claw.setPosition(0.6);
            }
        }
    }
    public void motionMagicDrive(double leftin, double rightin) {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double newFrontLeftTarget = frontLeft.getCurrentPosition() + (leftin * COUNTS_PER_INCH);
        double newFrontRightTarget = frontRight.getCurrentPosition() + (rightin * COUNTS_PER_INCH);
        double newBackLeftTarget = backLeft.getCurrentPosition() + (leftin * COUNTS_PER_INCH);
        double newBackRightTarget = frontRight.getCurrentPosition() + (rightin * COUNTS_PER_INCH);

//        double target = new MotionProfile();

        double rangeVariable = 10; //changed to help?

        while (opModeIsActive() && (runtime.seconds() < Math.abs(leftin)/10 ) && !(
                ((frontRight.getCurrentPosition() >= newFrontRightTarget-rangeVariable) && (rangeVariable+newFrontRightTarget >= frontRight.getCurrentPosition())) &&
                        ((frontLeft.getCurrentPosition() >= newFrontLeftTarget-rangeVariable) && (rangeVariable+newFrontLeftTarget >= frontLeft.getCurrentPosition())) &&
                        ((backLeft.getCurrentPosition() >= newBackLeftTarget-rangeVariable) && (rangeVariable+newBackLeftTarget >= backLeft.getCurrentPosition())) &&
                        ((backRight.getCurrentPosition() >= newBackRightTarget-rangeVariable) && (rangeVariable+newBackRightTarget >= backRight.getCurrentPosition()))
        ) ) {

        }
    }
}