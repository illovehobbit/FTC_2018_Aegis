//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@Autonomous(name="FTC_Deerfield_Autonomous_Parking")

public class FTC_Autonomous_Parking extends OpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = null;
    private ElapsedTime runtime_2 = null;
    private ElapsedTime runtime_3 = null;
    private DcMotor leftFront, rightFront, leftBack, rightBack, mastLift = null;
    private DcMotor armMotor, jointMotor = null;

    private double left_stick_y, time, time_2, time_3 = 0;

    private boolean unfolded;
    private boolean allClear = false;
    private boolean readyToPark = false;

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

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public void start() {  } //runtime.reset();

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // 1. Unfold back motor only.
        // Negative input is out
        // front motor control

        // =========================================================================================
        // back motor control
        int backJointPos = armMotor.getCurrentPosition();
        final double backPower = -0.50;

        if (backJointPos < 2000)
            armMotor.setPower(-backPower);
        else {
            unfolded = true;
            left_stick_y = -0.00;
        }

        // =========================================================================================
        // 2.1 Lower
        if (unfolded){

            // ------------------ mast lift control -------------------
            int liftPos = mastLift.getCurrentPosition();

            if (liftPos > -13400) {

                armMotor.setPower(0);

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
                setMotorPower(v1, v2, v3, v4);
            }
            // =====================================================================================
            // 2.2 Move sideways
            else if (!allClear) {

                // stop all motors
                mastLift.setPower(0);

                // initialize timer
                if (runtime == null) {
                    runtime = new ElapsedTime();
                    time = runtime.time();
                }
                // move side ways
                double current_time = runtime.time();
                if ((current_time - time) < 0.50){

                    armMotor.setPower(0);

                    telemetry.addData("moving sideways: ", (current_time - time));
                    double r = Math.hypot(-0.5, 0);
                    double robotAngle = Math.atan2(0, -0.5) - Math.PI/4;
                    double rightX = -0;

                    //calculate velocities of each wheel
                    final double v1 = r * Math.cos(robotAngle) + rightX;
                    final double v2 = r * Math.sin(robotAngle) - rightX;
                    final double v3 = r * Math.sin(robotAngle) + rightX;
                    final double v4 = r * Math.cos(robotAngle) - rightX;

                    //run each motor according to speed
                    setMotorPower(v1, v2, v3, v4);

                }else if ((current_time - time) > 0.50 && (current_time - time) < 0.90) {

                    armMotor.setPower(0);

                    double r = Math.hypot(0.5, 0);
                    double robotAngle = Math.atan2(0, 0.5) - Math.PI / 4;
                    double rightX = -0.5;

                    //calculate velocities of each wheel
                    final double v1 = r * Math.cos(robotAngle) + rightX;
                    final double v2 = r * Math.sin(robotAngle) - rightX;
                    final double v3 = r * Math.sin(robotAngle) + rightX;
                    final double v4 = r * Math.cos(robotAngle) - rightX;

                    //run each motor according to speed
                    setMotorPower(v1, v2, v3, v4);

                }else{
                    stopMotors();
                    allClear = true;
                    runtime = null;
                }
            }
        }

        // =========================================================================================
        // 3. After all clear, move forward
        if (allClear){
            if (runtime_2 == null){
                runtime_2 = new ElapsedTime();
                time_2 = runtime_2.time();
            }

            double current_time = runtime_2.time();
            if ((current_time - time_2) < 2.5){

                armMotor.setPower(0);

                telemetry.addData("time: ", (current_time - time_2));
                double r = Math.hypot(0, -0.5);
                double robotAngle = Math.atan2(-0.5, 0) - Math.PI/4;
                double rightX = -0;

                //calculate velocities of each wheel
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                //run each motor according to speed
                setMotorPower(v1, v2, v3, v4);

            }else{
                stopMotors();
                readyToPark = true;
            }
        }

        // Move arm and joint out parking sequence =================================================
        if (readyToPark){

            // moving joint motor
            int frontJointPos = jointMotor.getCurrentPosition();
            if (frontJointPos < 4000){
                jointMotor.setPower(1.0);
            }else{
                jointMotor.setPower(0);
            }

            // moving arm motor
            backJointPos = armMotor.getCurrentPosition();
            if (backJointPos < 3800){
                armMotor.setPower(0.80);
            }else{
                stopMotors();
                armMotor.setPower(0);
            }
        }
        telemetry.update();
    }

    private void stopMotors() {
        leftFront.setPower  (0);
        rightFront.setPower (0);
        leftBack.setPower   (0);
        rightBack.setPower  (0);
    }

    private void setMotorPower(double lfPower, double rfPower, double lbPower, double rbPower){
        leftFront.setPower  (lfPower);
        rightFront.setPower (rfPower);
        leftBack.setPower   (lbPower);
        rightBack.setPower  (rbPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
