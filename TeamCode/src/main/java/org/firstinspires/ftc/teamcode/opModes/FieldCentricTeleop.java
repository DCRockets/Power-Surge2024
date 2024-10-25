package org.firstinspires.ftc.teamcode.opModes;
import org.firstinspires.ftc.teamcode.subsystems.ComputerVision;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;

@TeleOp(name = "FieldCentric")
public class FieldCentricTeleop extends LinearOpMode {
    @Override public void runOpMode() {
        ComputerVision vision = new ComputerVision();
        vision.init(hardwareMap, telemetry);

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

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class,"armPivot");
//        DcMotorEx slidesMotor = hardwareMap.get(DcMotorEx.class,"slidesMotor");

        // setup movement motors
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // init motors at current position
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
//        slidesMotor.setTargetPosition((slidesMotor.getCurrentPosition()));
        //Init Speed and setpoints
        double pivotSpeed = 0;
//        double slidesSpeed = 0;
        int pivotSetpoint = armMotor.getCurrentPosition();
//        int slidesSetpoint = slidesMotor.getCurrentPosition();

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

        



        while(opModeIsActive()) {
            // get joystick values
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x; //negative left or negative right?




            // DRIVING ---------------------------------------------------------------------------

            // Read inverse imu heading
            double botHeading = imu.getAngularOrientation().firstAngle;
            if (botHeading > (2 * Math.PI)){
                botHeading -= (2 * Math.PI);
            } else if (botHeading < 0){
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


            // BUTTON BINDINGS -----------------------------------------

            // LINEAR SLIDES ------------

            //Button Bindings
//            if (gamepad1.dpad_up) {
  //              // slides high goal
    //            slidesSetpoint = 0;
      //          slidesSpeed = velocityDirection(slidesSetpoint, slidesMotor.getCurrentPosition(), 2);
        //    } else if (gamepad1.dpad_left) {
          //      // slides low goal
            //    slidesSetpoint = 0;
              //  slidesSpeed = velocityDirection(slidesSetpoint, slidesMotor.getCurrentPosition(), 2);
//            } else if (gamepad1.dpad_right) {
  //              // slides climbing
    //            slidesSetpoint = 0;
      //          slidesSpeed = velocityDirection(slidesSetpoint, slidesMotor.getCurrentPosition(), 2);
        //    } else if (gamepad1.dpad_down) {
          //      // slides ground
            //    slidesSetpoint = 0;
              //  slidesSpeed = velocityDirection(slidesSetpoint, slidesMotor.getCurrentPosition(), 2);
           // }

            // Set position and velocity
           // slidesMotor.setTargetPosition(slidesSetpoint);
           // slidesMotor.setVelocity(slidesSpeed);



            // PIVOT POSITIONS -----------------

            //Button Bindings
            if (gamepad1.y) {
                // pivot ground goal
                pivotSetpoint = 5000;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), 2);
            } else if (gamepad1.x) {
                // pivot low goal
                pivotSetpoint = 1700;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), 2);
            } else if (gamepad1.b) {
                // pivot climb
                pivotSetpoint = 4900;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), 2);
            } else if (gamepad1.a) {
                // pivot ground
                pivotSetpoint = 0;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), 2);
            }

            // Set position and velocity
            armMotor.setTargetPosition(pivotSetpoint);
            armMotor.setVelocity(pivotSpeed);



            // ARM PIVOT MANUAL - NOT USED ----------------------

            if (gamepad1.start) {
                pivotSetpoint += 30;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), -2);
            } else if (gamepad1.back) {
                pivotSetpoint -= 30;
                pivotSpeed = velocityDirection(pivotSetpoint, armMotor.getCurrentPosition(), -2);
            }


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
                wrist.setPosition(0.3);
            }
            if (gamepad1.dpad_right) {
                wrist.setPosition(0.2);
            }
            if (gamepad1.dpad_down) {
                wrist.setPosition(0);
            }
            if (gamepad1.dpad_up) {
                wrist.setPosition(-0.05);
            }




           /* if (gamepad1.dpad_up) {
                wrist.setPosition(0.5);
            } else if (gamepad1.dpad_down) {
                wrist.setPosition(0.4);
            }
*/            // TELEMETRY -------------------------------------------------------------------------

            telemetry.addData("arm velo set", pivotSpeed);
            telemetry.addData("arm current position", armMotor.getCurrentPosition());
            telemetry.addData("arm position goal", pivotSetpoint);
            telemetry.addData("Angle",botHeading);
            telemetry.addData("Rotx",rotX);
            telemetry.addData("roty",rotY);
            telemetry.addData("x",x);
            telemetry.addData("math.cos",Math.cos(botHeading));
            telemetry.addData("Wrist Pos", wrist.getPosition());
/*
            telemetry.addData("wrist pos", wrist.getPosition());
*/
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