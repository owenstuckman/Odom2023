package robotx.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import robotx.libraries.XModule;

public class OdomSystem extends XModule {

    //var setup
    public Servo odom1;
    public Servo odom2;
    public Servo odom3;
    public DigitalChannel touchSensor;

    boolean blocked = false;

    //methods are built into one button as a toggle

    public void OdomSys() {

        odom1.setPosition(0.27);
        odom2.setPosition(0.83);
        odom3.setPosition(0.12);

    }
    public void OdomSysUp() {

        odom1.setPosition(0.52);
        odom2.setPosition(0.11);
        odom3.setPosition(0.46);

    }
    public void Odomsystem() {
        if (!blocked) {
            odom1.setPosition(0.46);
            odom2.setPosition(0.11);
            odom3.setPosition(0.46);
            blocked = true;
        }
        else {
            odom1.setPosition(0.37);
            odom2.setPosition(0.77);
            odom3.setPosition(0.21);
            blocked = false;
        }
    }

    public OdomSystem (OpMode op) {
        super(op);
    }

    public void init() {
        // pulls servos from configs
        odom1 = opMode.hardwareMap.servo.get("frontOdom");
        odom2 = opMode.hardwareMap.servo.get("leftOdom");
        odom3 = opMode.hardwareMap.servo.get("rightOdom");
    }

    public void loop() {
        // button presses, calls methods
        if (xGamepad1().dpad_left.wasPressed()){
            odom1.setPosition(0.46);
            odom2.setPosition(0.11);
            odom3.setPosition(0.46);

        }
    }
}

// dpad down - odom wheel up/down on a toggle
// - gamepad 1
