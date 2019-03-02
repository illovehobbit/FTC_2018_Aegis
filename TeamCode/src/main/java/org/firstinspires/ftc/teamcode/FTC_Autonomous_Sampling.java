//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

@Autonomous(name="FTC_Deerfield_Autonomous_Sampling")

public class FTC_Autonomous_Sampling extends OpMode{

    // Declare OpMode members.
    private ModernRoboticsI2cColorSensor colorSensor;    // Hardware Device Object
    private ModernRoboticsI2cRangeSensor rangeSensor_left;
    private ModernRoboticsI2cRangeSensor rangeSensor_right;

    private ElapsedTime runtime_2 = null;
    private DcMotor leftFront, rightFront, leftBack, rightBack, mastLift = null;
    private DcMotor armMotor, jointMotor = null;

    private double left_stick_y = 0;

    private boolean unfolded;
    private boolean allClear = false;
    private boolean readyToSample = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        unfolded = false;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "sensor_color");
        rangeSensor_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_left");
        rangeSensor_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_right");

        colorSensor.enableLed(true);

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
            left_stick_y = -0.08;
        }

        // =========================================================================================
        // 2.1 Lower
        if (unfolded) {

            // ------------------ mast lift control -------------------
            int liftPos = mastLift.getCurrentPosition();

            if (liftPos > -14000) {

                armMotor.setPower(0);

                mastLift.setPower(-1);

                double r = Math.hypot(0, left_stick_y);
                double robotAngle = Math.atan2(left_stick_y, 0) - Math.PI / 4;
                double rightX = -0;

                //calculate velocities of each wheel
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                //run each motor according to speed
                setMotorPower(v1, v2, v3, v4);
            } else {
                allClear = true;
                mastLift.setPower(0);
            }
        }

        // =========================================================================================
        // 3. After all clear, move forward
        double distance_reading_right = rangeSensor_right.getDistance(DistanceUnit.CM);
        double distance_reading_left = rangeSensor_left.getDistance(DistanceUnit.CM);

        if (allClear){

            if (distance_reading_right > 12 && distance_reading_left > 12 && readyToSample == false){
                armMotor.setPower(-backPower);

            }else{
                armMotor.setPower(0);
                mastLift.setPower(0);
                stopMotors();
                readyToSample = true;
            }
        }

        // Move arm and joint out parking sequence =================================================
        if (readyToSample){


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
