package robotx.modules;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import robotx.libraries.XModule;
import robotx.opmodes.autonomous.RobotXTester;


public class LiftMotors extends XModule {


    // motors being used

    public DcMotor LeftLift1;
    public DcMotor LeftLift2;

    public DcMotor RightLift1;
    public DcMotor RightLift2;

    public Servo gearServo;

    double power = 1;
    double power2 = 1;
    boolean blocked;

    public TouchSensor touch;

    public ColorSensor colorSensor;

    public LiftMotors(OpMode op) {
        super(op);
    }

    public void init() {

        //init motors from configs

        LeftLift1 = opMode.hardwareMap.dcMotor.get("leftLift1");
        LeftLift2 = opMode.hardwareMap.dcMotor.get("leftLift2");
        RightLift1 = opMode.hardwareMap.dcMotor.get("rightLift1");
        RightLift2 = opMode.hardwareMap.dcMotor.get("rightLift2");
        gearServo = opMode.hardwareMap.servo.get("gearLockServo");

        touch = opMode.hardwareMap.touchSensor.get("touchSensor");

        colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");


    }

    public void gearLock() {
        if (!blocked) {
            gearServo.setPosition(0.5);
            blocked = true;
        } else {
            gearServo.setPosition(0.88);
            blocked = false;
        }
    }


    public void loop() {

        if (xGamepad2().dpad_left.wasPressed()) {
            gearLock();
        }
        if (xGamepad2().a.isDown()) {
            LeftLift1.setPower(1);
            LeftLift2.setPower(1);
            RightLift1.setPower(1);
            RightLift2.setPower(1);
        } else if (xGamepad2().b.isDown()) {
            LeftLift1.setPower(-0.65);
            LeftLift2.setPower(-0.65);
            RightLift2.setPower(-0.65);
            RightLift1.setPower(-0.65);
        }
       else {
            LeftLift1.setPower(0);
            LeftLift2.setPower(0);
            RightLift1.setPower(0);
            RightLift2.setPower(0);
        }

        if (xGamepad2().dpad_up.isDown()) {
            LeftLift1.setPower(.4);
            LeftLift2.setPower(.4);
            RightLift2.setPower(.4);
            RightLift1.setPower(.4);
        } else if (xGamepad2().dpad_down.isDown()) {
            LeftLift1.setPower(-.4);
            LeftLift2.setPower(-.4);
            RightLift1.setPower(-.4);
            RightLift2.setPower(-.4);
        }
        else {
            LeftLift1.setPower(0.0);
            RightLift2.setPower(0);
            RightLift1.setPower(0);
            LeftLift2.setPower(0);
        }
    }
}


/*
a - all motors same direction
b - all motors opposite direction
 */
