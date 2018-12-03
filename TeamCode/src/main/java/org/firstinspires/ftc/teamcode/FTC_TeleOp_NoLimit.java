/*
 * 201-2019 Deerfield Academy FTC Robot Code
 * By Will Song '20 & Neil Nie '19
 * (c) Deerfield Academy, All Rights Reserved
 * Contact: ynie19@deerfield.edu
 *          jsong20@deerfield.edu
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FTC_TeleOp_NoLimit", group="Iterative Opmode")

public class FTC_TeleOp_NoLimit extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, mastLift = null;
    private DcMotor armMotor, jointMotor, collectorMotor, dumpMotor = null;

    private final double dumpSpeed = 0.05;
    private final double drivingSpeedScaling = 0.5;


    @Override
    public void init() {

        // Initialize wheel drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Set direction for wheel drive motors
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Initialize mast lift motor
        mastLift = hardwareMap.get(DcMotor.class, "mast_lift");
        mastLift.setDirection(DcMotor.Direction.FORWARD);                   // Set direction for motor - need to test
        mastLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // when at zero power, motor actively brakes

        // Init arm motor
        armMotor = hardwareMap.get(DcMotor.class, "back_motor");
        armMotor.setDirection(DcMotor.Direction.FORWARD);                   // set direction of motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // set braking behavior

        // Init arm joint motor
        jointMotor = hardwareMap.get(DcMotor.class, "front_motor");
        jointMotor.setDirection(DcMotor.Direction.FORWARD);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init collector motor
        collectorMotor = hardwareMap.get(DcMotor.class, "collector_motor");
        collectorMotor.setDirection(DcMotor.Direction.FORWARD);

        // Init Servo
        dumpMotor = hardwareMap.get(DcMotor.class, "dump_motor");
        dumpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // set braking behavior

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // -------------------- Mecanum driving --------------------
        //read input values for driving (Gamepad 1)
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x) - Math.PI/4;
        double rightX = -gamepad1.right_stick_x;

        //calculate velocities of each wheel
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //run each motor according to speed
        leftFront.setPower  (v1 * drivingSpeedScaling);
        rightFront.setPower (v2 * drivingSpeedScaling);
        leftBack.setPower   (v3 * drivingSpeedScaling);
        rightBack.setPower  (v4 * drivingSpeedScaling);

        // ------------------ mast lift control -------------------

        boolean up = gamepad1.right_bumper;
        boolean down = gamepad1.left_bumper;

        if (down && !up)
            mastLift.setPower(1);
        else if (up && !down)
            mastLift.setPower(-1);
        else
            mastLift.setPower(0);

        // ------------------ dump motor control -------------------
        int dumpPosition = dumpMotor.getCurrentPosition();
        if (gamepad1.y && dumpPosition < 750)
            dumpMotor.setPower(0.45);
        else if (gamepad1.y && dumpPosition < 780 && dumpPosition > 770)
            dumpMotor.setPower(0.01);
        else if (gamepad1.a && dumpPosition > 560)
            dumpMotor.setPower(-0.20);
        else if (gamepad1.a && dumpPosition < 135 && dumpPosition > 50)
            dumpMotor.setPower(-0.005);
        else
            dumpMotor.setPower(0);

        // ------------------ front motor control -------------------
        int frontJointPos = jointMotor.getCurrentPosition();
        final double frontPower = Range.scale(gamepad2.right_stick_y, -1, 1, -0.25, 0.25);

        if (frontPower < 0)
            jointMotor.setPower(-frontPower);
        else if (frontPower > 0)
            jointMotor.setPower(-frontPower);
        else
            jointMotor.setPower(0);

        // ------------------ back motor control -----------------
        int backJointPos = armMotor.getCurrentPosition();
        final double backPower = Range.scale(gamepad2.left_stick_y, -1, 1, -0.25, 0.25);

        if (backPower < 0)
            armMotor.setPower(-backPower);
        else if (backPower > 0)
            armMotor.setPower(-backPower);
        else
            armMotor.setPower(0);

        // --------------- collector motor control ----------------
        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            collectorMotor.setPower(1);
        }else if (gamepad2.left_bumper && !gamepad2.right_bumper){
            collectorMotor.setPower(-1);
        }else{
            collectorMotor.setPower(0);
        }

        try {
            Thread.sleep(50);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    @Override
    public void stop() { }
}