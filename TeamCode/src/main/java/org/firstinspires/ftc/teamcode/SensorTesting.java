/*
 * 201-2019 Deerfield Academy FTC Robot Code
 * By Will Song '20 & Neil Nie '19
 * (c) Deerfield Academy, All Rights Reserved
 * Contact: ynie19@deerfield.edu
 *          jsong20@deerfield.edu
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Sensor_Testing")

public class SensorTesting extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ModernRoboticsI2cColorSensor colorSensor;    // Hardware Device Object
    private ModernRoboticsI2cRangeSensor rangeSensor;

    private DcMotor leftFront, rightFront, leftBack, rightBack, mastLift = null;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F,0F,0F};

    private boolean bLedOn = true;
    // values is a reference to the hsvValues array.
    private final float values[] = hsvValues;

    @Override
    public void init() {

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "sensor_color");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

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


        double distance_reading = rangeSensor.getDistance(DistanceUnit.CM);

        if (distance_reading > 10){

            double r = Math.hypot(0, -0.3);
            double robotAngle = Math.atan2(-0.3, 0) - Math.PI/4;
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
        }

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Status", "Run Time: " + runtime.toString());

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

    @Override
    public void stop() { }
}