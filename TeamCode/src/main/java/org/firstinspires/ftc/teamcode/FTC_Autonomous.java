//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="FTC_Deerfield_Autonomous")

public class FTC_Autonomous extends OpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, mastLift = null;
    private DcMotor armMotor, jointMotor = null;

    private double left_stick_y, time = 0;

    private boolean unfolded;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        unfolded = false;

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

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() { runtime.reset(); }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // 1. Unfold back motor only.
        // Negative input is out
        // front motor control

        // back motor control
        int backJointPos = armMotor.getCurrentPosition();
        final double backPower = -0.30;

        if (backJointPos < 2000)
            armMotor.setPower(-backPower);
        else {
            armMotor.setPower(0);
            unfolded = true;
            left_stick_y = -0.15;
        }


        // -------------------- Mecanum driving --------------------
        //read input values for driving (Gamepad 1)

        // 2.1 Lower
        if (unfolded){

            // ------------------ mast lift control -------------------
            int liftPos = mastLift.getCurrentPosition();

            if (liftPos > -12000) {
                mastLift.setPower(-1);

                double r = Math.hypot(0, left_stick_y);
                double robotAngle = Math.atan2(left_stick_y, 0) - Math.PI/4;
                double rightX = -0;

                //calculate velocities of each wheel
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                //run each motor according to speed
                leftFront.setPower  (v1);
                rightFront.setPower (v2);
                leftBack.setPower   (v3);
                rightBack.setPower  (v4);
            }
            else {
                mastLift.setPower(0);

                leftFront.setPower  (0);
                rightFront.setPower (0);
                leftBack.setPower   (0);
                rightBack.setPower  (0);

                time = runtime.time();
            }

        }

        if ((runtime.time() - time) < 3e+9){

            double r = Math.hypot(-0.20, 0);
            double robotAngle = Math.atan2(0, -0.20) - Math.PI/4;
            double rightX = -0;

            //calculate velocities of each wheel
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            //run each motor according to speed
            leftFront.setPower  (v1);
            rightFront.setPower (v2);
            leftBack.setPower   (v3);
            rightBack.setPower  (v4);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        // 2.2 Run motor

        // Translate Left

        // Move Forwards
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
