package org.firstinspires.ftc.teamcode.opModes;
//import org.firstinspires.ftc.teamcode.subsystems.ComputerVision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.teamcode.utility.MotionProfile;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.0075, i = 0.05, d = 0;
    public static double f = 0;
    private DcMotor armMotor;
    private PIDController controller2;
    public static double p2 = .00075, i2 = 0.075, d2 = 0;
    public static double f2 = 0;
    private DcMotor elbowMotor;

    public static int target = 0;
    public static int target2 = 0;

    private final double ticks_in_degree = 1120 / 270;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double pivotJerk = 3000;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime runtime = new ElapsedTime();


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeft = hardwareMap.dcMotor.get("rightFront");
        DcMotor frontRight = hardwareMap.dcMotor.get("leftRear");
        DcMotor backRight = hardwareMap.dcMotor.get("rightRear");

        // Reverse the right side motors
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        CRServo intake = hardwareMap.get(CRServo.class, "Intake");
        Servo wrist = hardwareMap.get(Servo.class, "Wrist");
        Servo claw = hardwareMap.get(Servo.class, "Claw");

        double wristPosition = wrist.getPosition();

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armPivot");
        DcMotorEx elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");

        // setup movement motors


        // Initialize imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();


        runtime.reset();
        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            armMotor.setPower(power);

            controller2.setPID(p2, i2, d2);
            int elbowPos = elbowMotor.getCurrentPosition();
            double pid2 = controller2.calculate(elbowPos, target2);
            double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f2;
            double power2 = pid2 + ff2;
            elbowMotor.setPower(power2);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.addData("pos2", elbowPos);
            telemetry.addData("target2", target2);
            telemetry.addData("FL" , frontLeft.getCurrentPosition());
            telemetry.addData("FR" , frontRight.getCurrentPosition());
            telemetry.addData("BL" , backLeft.getCurrentPosition());
            telemetry.addData("BR" , backRight.getCurrentPosition());
            telemetry.update();

            if (gamepad1.y) {
                target = -1200;
                target2 = 1200;
            }
            if (gamepad1.x) {
                target = -2140;
                target2 = 1215;
            }
            if (gamepad1.dpad_left) {
                target = 0;
                target2 = 0;
            }
            if (gamepad1.dpad_right) {
                armMotor.setPower(-1);
                target = armMotor.getCurrentPosition();
            }

            if (gamepad1.start) {
                target += 25;
            } else if (gamepad1.back) {
                target -= 25;
            }

            if (gamepad1.right_trigger >= 0.95) {
                target2 -= 10;
            } else if (gamepad1.left_trigger >= 0.95) {
                target2 += 10;
            }


            // arm limits
            if ((target < -7500)||(target > 0))  {
                if (target < -7500) {
                    target = -7500;
                } else if (target > 0) {
                    target = 0;
                }

            }
            // get joystick values
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x; //negative left or negative right?


            // DRIVING ---------------------------------------------------------------------------

            // Read inverse imu heading
            double botHeading = imu.getAngularOrientation().firstAngle;
            if (botHeading > (2 * Math.PI)) {
                botHeading -= (2 * Math.PI);
            } else if (botHeading < 0) {
                botHeading += (2 * Math.PI);
            }


            // Rotate gamepad input x/y by hand
            // See: https://matthew-brett.github.io/teaching/rotation_2d.html
            double rotX = (x * Math.sin(botHeading)) + (y * Math.cos(botHeading));
            double rotY = (y * Math.sin(botHeading)) - (x * Math.cos(botHeading));

            // assign wheel power
            double frontLeftPower = -rotX - rotY + turn;
            double backLeftPower = -rotX + rotY - turn;
            double frontRightPower = -rotX + rotY + turn;
            double backRightPower = -rotX - rotY - turn;

            // Put powers in the range of -1 to 1 only if they aren't already
            // Not checking would cause us to always drive at full pivotSpeed
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

            double pos = 0;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            if (gamepad1.b) {
                runtime.reset();
                claw.setPosition(0.6);
            } else if (gamepad1.a) {
                claw.setPosition(0.9);
                runtime.reset();
            }
            if (claw.getPosition() == 0.9) {
                sleep(100);
                target = -1650;
            }
            if (gamepad1.left_bumper) {
                intake.setPower(0.5);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-0.5);
            } else {
                intake.setPower(-0.1);
            }
            if (gamepad1.dpad_down) {
                wristPosition -= 0.01;
            } else if (gamepad1.dpad_up) {
                wristPosition += 0.01;
            }

            if ((wristPosition < 0)||(wristPosition > 1))  {
                if (wristPosition < 0) {
                    wristPosition = 0;
                } else if (wristPosition > 1) {
                    wristPosition = 1;
                }

            }
            wrist.setPosition(wristPosition);
        }
    }
}