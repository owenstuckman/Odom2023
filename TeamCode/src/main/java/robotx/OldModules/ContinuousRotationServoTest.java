package robotx.OldModules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import robotx.libraries.XModule;

/**
 * Created by Ben Sabo on 9/18/2017.
 */

public class ContinuousRotationServoTest extends XModule {

    CRServo rotationServo;

    public ContinuousRotationServoTest(OpMode op) {
        super(op);
    }

    public void init(){
        rotationServo = opMode.hardwareMap.crservo.get("rotationServo");
    }

    public void loop(){

        float rotationPower = xGamepad1().left_stick_y;
        rotationPower /= 0.5;
        rotationPower += 0.5;
        rotationServo.setPower(rotationPower);
        double currentSpeed = xGamepad1().left_stick_y;
        opMode.telemetry.addData("Servo Speed:", currentSpeed);
    }

}