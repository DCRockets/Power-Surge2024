package org.firstinspires.ftc.teamcode.opModes;
//import org.firstinspires.ftc.teamcode.subsystems.ComputerVision;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.teamcode.utility.MotionProfile;

@TeleOp(name = "FieldCentric")
@Disabled
public class FieldCentricTeleop extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double pivotJerk = 3000;

    @Override public void runOpMode() {


        ElapsedTime runtime = new ElapsedTime();


//        ComputerVision vision = new ComputerVision();
//        vision.init(hardwareMap, telemetry);

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("frontRight");
        DcMotor frontRight = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        // Reverse the right side motors
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        CRServo intake = hardwareMap.get(CRServo.class, "Intake");
        Servo wrist = hardwareMap.get(Servo.class, "Wrist");
        Servo claw = hardwareMap.get(Servo.class, "Claw");

        double wristPosition = wrist.getPosition();

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class,"armPivot");
        DcMotorEx elbowMotor = hardwareMap.get(DcMotorEx.class,"elbowMotor");

        // setup movement motors
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // init motors at current position
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        elbowMotor.setTargetPosition(elbowMotor.getCurrentPosition());

//        elbowMotor.setTargetPosition((elbowMotor.getCurrentPosition()));
        //Init Speed and setpoints
        double pivotSpeed = 0;
        double elbowSpeed = 0;
        int pivotSetpoint = armMotor.getCurrentPosition();
        int elbowSetpoint = elbowMotor.getCurrentPosition();

        MotionProfile target = new MotionProfile(10, 4 * COUNTS_PER_INCH,pivotSetpoint);
        MotionProfile target2 = new MotionProfile(10, 4*COUNTS_PER_INCH,elbowSetpoint);


        // testing PIDF
        PIDFController armPIDF = new PIDFController(1,2,3,4);
        armPIDF.setP(0.9);
        armPIDF.setI(0.5);
        armPIDF.setD(0.1);
        armPIDF.setF(0.01);


        // Initialize imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        


        runtime.reset();
        while(opModeIsActive()) {
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


            // PIVOT POSITIONS -----------------

            //Button Bindings
            if (gamepad1.y) {
                // specimin grab
                armMotor.setTargetPosition(-700);
//                pivotSetpoint = -700;
//                target = new MotionProfile(pivotJerk, 9 * COUNTS_PER_INCH, pivotSetpoint - armMotor.getCurrentPosition());
                elbowMotor.setTargetPosition(-1070);
//                elbowSetpoint = -1070;
//                target = new MotionProfile(pivotJerk, 9 * COUNTS_PER_INCH, elbowSetpoint - elbowMotor.getCurrentPosition());
            } else if (gamepad1.x) {
                //specimin score
                armMotor.setTargetPosition(-2171);
//                pivotSetpoint = -2171;
//                target = new MotionProfile(pivotJerk, 9 * COUNTS_PER_INCH, pivotSetpoint - armMotor.getCurrentPosition());
                elbowMotor.setTargetPosition(-1240);
//                elbowSetpoint = -1240;
//                target = new MotionProfile(pivotJerk, 9 * COUNTS_PER_INCH, elbowSetpoint - elbowMotor.getCurrentPosition());

            } else if (gamepad1.b) {
                // pivot climb

//                pivotSetpoint = 4900;
//                target = new MotionProfile(pivotJerk, 4 * COUNTS_PER_INCH,pivotSetpoint);
                runtime.reset();
                claw.setPosition(0.4);
            } else if (gamepad1.a) {
                // pivot ground
//                pivotSetpoint = 0;
//                target = new MotionProfile(pivotJerk, 4 * COUNTS_PER_INCH,pivotSetpoint);
                claw.setPosition(0.7);
                runtime.reset();
            }

            if (claw.getPosition() == 0.7 && pivotSetpoint >= -1000) {
                pivotSetpoint = -1000;
            }

            if (target != null) {
                double[] result = target.computeMotion(runtime.seconds());
                pivotSpeed = result[1];
            }

            // ARM PIVOT MANUAL -----------------------------------------

            if (gamepad1.start) {
                pivotSetpoint += 30;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), -1);
                target = null;
            } else if (gamepad1.back) {
                pivotSetpoint -= 30;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), -1);
                target = null;
            }

            if ((pivotSetpoint - 4 <= armMotor.getCurrentPosition() && armMotor.getCurrentPosition() <= pivotSetpoint + 4) && target != null) {
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), -2);
                target = null;
            }



            // Set position and velocity
            armMotor.setTargetPosition(pivotSetpoint);
            armMotor.setVelocity(pivotSpeed);


            // INTAKE BUTTONS -------------------------------

            if (gamepad1.left_bumper) {
                intake.setPower(0.5);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-0.5);
            } else {
                intake.setPower(-0.1);
            }

            // WRIST BUTTONS ------------------------------------------
//            if (gamepad1.dpad_right) {
//                wrist.setPower(0.1);
//            } else if (gamepad1.dpad_left) {
//                wrist.setPower(-0.1);
//            } else {
//                wrist.setPower(0.05);
//            }

            if (gamepad1.dpad_left) {
                elbowSetpoint -= 10;
                elbowSpeed = velocityDirection(elbowSetpoint, elbowMotor.getCurrentPosition(), -1);
                target = null;
            } else if (gamepad1.dpad_right) {
                elbowSetpoint += 10;
                elbowSpeed = velocityDirection(elbowSetpoint, elbowMotor.getCurrentPosition(), -1);
                target = null;
            }

            if ((elbowSetpoint - 4 <= elbowMotor.getCurrentPosition() && elbowMotor.getCurrentPosition() <= elbowSetpoint + 4) && target != null) {
                elbowSpeed = velocityDirection(elbowSetpoint, elbowMotor.getCurrentPosition(), -2);
                target = null;
            }
            elbowMotor.setTargetPosition(elbowSetpoint);
            elbowMotor.setVelocity(elbowSpeed);


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





           /* if (gamepad1.dpad_up) {
                wrist.setPosition(0.5);
            } else if (gamepad1.dpad_down) {
                wrist.setPosition(0.4);
            }
*/            // TELEMETRY -------------------------------------------------------------------------

            telemetry.addData("arm velo set", pivotSpeed);
            telemetry.addData("arm current position", armMotor.getCurrentPosition());
            telemetry.addData("arm position goal", pivotSetpoint);
            telemetry.addData("Angle", botHeading);
            telemetry.addData("Rotx", rotX);
            telemetry.addData("roty", rotY);
            telemetry.addData("x", x);
            telemetry.addData("math.cos", Math.cos(botHeading));
            telemetry.addData("Wrist Pos", wrist.getPosition());
            telemetry.addData("Wrist setpos", wristPosition);
            telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("elbow setpos", elbowSetpoint);
            telemetry.addData("claw pos", claw.getPosition());
            if (target != null) {
                telemetry.addData("t1", target.timeIntervals[0]);
                telemetry.addData("t2", target.timeIntervals[1]);
                telemetry.addData("t3", target.timeIntervals[2]);
                telemetry.addData("test", target.e);
            }
            telemetry.update();

        }
    }

    public double velocityDirection(int setpoint, int currentPosition, double speed) {
        // ticks per motor rotation without gearbox: 28
        // arm gearbox: 50.9
        // 5:1 big gear

        // speed is in rps from the motor (28*50.9)

        speed = speed * 50.9 * 28;
        if (currentPosition != setpoint) {
            if (currentPosition > setpoint) {
                speed = Math.abs(speed) * -1;
            } else if (currentPosition < setpoint) {
                speed = Math.abs(speed);
            }
        } else {
            speed = 0;
        }

        return speed;
    }
}