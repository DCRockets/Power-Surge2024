package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FieldCentric")
public class FieldCentricTeleop extends LinearOpMode {
    @Override public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("frontRight");
        DcMotor frontRight = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        // Reverse the right side motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor armMotor = hardwareMap.dcMotor.get("armPivot");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        PIDFController armPIDF = new PIDFController(1,2,3,4);
        // Initialize imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Read inverse imu heading
            double botHeading = imu.getAngularOrientation().firstAngle;
            telemetry.addData("Angle",botHeading);
            telemetry.update();

            // Rotate gamepad input x/y by hand
            // See: https://matthew-brett.github.io/teaching/rotation_2d.html
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = y * Math.sin(botHeading) - x * Math.cos(botHeading);

            double frontLeftPower = rotY + rotX + turn;
            double backLeftPower = rotY - rotX + turn;
            double frontRightPower = rotY - rotX - turn;
            double backRightPower = rotY + rotX - turn;

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

            if (gamepad1.dpad_up) {
                armMotor.setPower(0.1);
            } else if (gamepad1.dpad_down) {
                armMotor.setPower((-0.1));

            } else {
                armMotor.setPower(0);
            }
            // update pid controller
            armPIDF.setSetPoint(1200);

            // perform the control loop
            /*
             * The loop checks to see if the controller has reached
             * the desired setpoint within a specified tolerance
             * range
             */
//            double output = armPIDF.calculate(
//                    armMotor.getCurrentPosition()  // the measured value
//            );
//            armMotor.setPower(output);
//            armMotor.setPower(0);

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

        }
    }
}