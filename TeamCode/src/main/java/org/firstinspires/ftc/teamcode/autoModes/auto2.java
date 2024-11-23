package org.firstinspires.ftc.teamcode.autoModes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="NEW TIME AUTO", group="Robot")
public class auto2 extends LinearOpMode {


    /* Declare OpMode members. */
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotorEx armMotor = null;
    private DcMotorEx elbowMotor = null;

    private ElapsedTime runtime = new ElapsedTime();

    private final double ticks_in_degree = 1120 / 270;
    private PIDController controller;
    public static double p = 0.01, i = 0.05, d = 0;
    public static double f = 0;
    private PIDController controller2;
    public static double p2 = .00075, i2 = 0.075, d2 = 0;
    public static double f2 = 0;


    static final double     FORWARD_SPEED = 0.2;
    static final double     TURN_SPEED    = 0.2;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);

        // Initialize the drive system variables.
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armPivot");
        DcMotorEx elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(-FORWARD_SPEED);
        backLeft.setPower(-FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
//        frontLeft.setPower(-TURN_SPEED);
//        frontRight.setPower(-TURN_SPEED);
//        backLeft.setPower(TURN_SPEED);
//        backRight.setPower(TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3)) {
//            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        frontLeft.setPower(FORWARD_SPEED);
//        frontRight.setPower(-FORWARD_SPEED);
//        backLeft.setPower(-FORWARD_SPEED);
//        backRight.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(500);


//        // Step 3:  Drive Backward for 1 Second
//        frontLeft.setPower(-TURN_SPEED);
//        frontRight.setPower(-TURN_SPEED);
//        backLeft.setPower(TURN_SPEED);
//        backRight.setPower(TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
//            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }


        // Step 4:  Stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)){
            double armTarget = -1200;
            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
            double power = pid + ff;
            armMotor.setPower(power);
        }
        sleep(1000);

        while (opModeIsActive() && (runtime.seconds() < 1)) {
            double target2 = -600;
            controller2.setPID(p2, i2, d2);
            int elbowPos = elbowMotor.getCurrentPosition();
            double pid2 = controller2.calculate(elbowPos, target2);
            double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f2;
            double power2 = pid2 + ff2;
            elbowMotor.setPower(power2);
        }



        telemetry.update();
        sleep(10000);
    }
}
