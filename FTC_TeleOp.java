
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="FTC_TeleOp", group="Iterative Opmode")
//@Disabled
public class FTC_TeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, mastLift = null;
    private int liftPos;
    private Servo dumpServo = null;

    private double dumpPos;
    private final double dumpSpeed = 0.05;

    @Override
    public void init() {

        //Initialize wheel drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        //Set direction for wheel drive motors
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        //Run wheel drive motors based on speed, not power - need setMaxSpeed
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize mast life motor
        mastLift = hardwareMap.get(DcMotor.class, "mast_lift");
        mastLift.setDirection(DcMotor.Direction.FORWARD);//Set direction for motor - need to test
        mastLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//when at zero power, motor actively brakes

        //Init Servo
        dumpServo = hardwareMap.get(Servo.class, "dump_servo");
        dumpPos = 0;
        dumpServo.setPosition(dumpPos);

        //update telemetry
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
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);


        double lift = gamepad2.right_stick_y;
        final double liftPower = Range.clip(lift, -1.0, 1.0);

        liftPos = mastLift.getCurrentPosition();

        if (liftPower > 0 && liftPos < 0){

            mastLift.setPower(liftPower);
        }
        else if (liftPower < 0 && liftPos > -14000){

            mastLift.setPower(liftPower);
        } else {

            mastLift.setPower(0);
        }




        if (gamepad2.y && dumpPos < 1) {

            dumpPos += dumpSpeed;
        } else if (gamepad2.a && dumpPos > 0) {

            dumpPos -= dumpSpeed;
        }

        dumpServo.setPosition(dumpPos);
//
//        // just testing the servo
//        if (gamepad2.x){
//            dumpServo.setPosition(0);
//        }
//        if (gamepad2.b){
//            dumpServo.setPosition(1);
//        }
//        if (gamepad2.a){
//            dumpServo.setPosition(-1);
//        }




        try {
            Thread.sleep(50);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }


        telemetry.addData("Encoder: ", liftPos);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    @Override
    public void stop() { }
}
