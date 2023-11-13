package robotx.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import robotx.libraries.XModule;

public class ClawSystem extends XModule {

    //var setup
    public Servo leftArm;
    public Servo rightArm;
    public Servo claw;
    boolean blocked = false;
    boolean blocked2 = false;
    boolean closed;

    //methods are built into one button as a toggle

    // method for moving the four bar
    public void leftArmRightArm() {
        if (!blocked) {
            leftArm.setPosition(.36);
            rightArm.setPosition(.696);
            blocked = true;
        }
        else {
            leftArm.setPosition(0.62);
            rightArm.setPosition(0.42);
            blocked = false;
        }
    }
    // method for the claw
     public void claw() {
        if (!blocked2) {
            claw.setPosition(0);
            blocked2 = true;
            closed = true;
        }
        else {
            claw.setPosition(0.2);
            blocked2 = false;
            closed = false;
        }
    }



    public ClawSystem (OpMode op) {
        super(op);
    }

    public void init() {
        // pulls servos from configs
        leftArm = opMode.hardwareMap.servo.get("leftArm");
        rightArm = opMode.hardwareMap.servo.get("rightArm");
        claw = opMode.hardwareMap.servo.get("claw");
    }

    public void loop() {
        // button presses, calls methods

        if (!closed) {
            if (xGamepad2().x.wasPressed()) {
                leftArmRightArm();
            }
        }
        if (xGamepad2().y.wasPressed()){
            claw();
        }

    }
}

// x - four bar up/down
// y - claw open/close
