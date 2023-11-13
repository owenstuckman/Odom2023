package robotx.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import robotx.libraries.XModule;




public class EncoderTester extends XModule {

    public DcMotor LeftLift1;
    public DcMotor LeftLift2;

    public DcMotor RightLift1;
    public DcMotor RightLift2;

    public EncoderTester(OpMode op) {
        super(op);
    }

    //@Override
    public void runOpMode() throws InterruptedException
    {
        //call motors from configs
        LeftLift1 = opMode.hardwareMap.dcMotor.get("leftLift1");
        LeftLift2 = opMode.hardwareMap.dcMotor.get("leftLift2");
        RightLift1 = opMode.hardwareMap.dcMotor.get("rightLift1");
        RightLift2 = opMode.hardwareMap.dcMotor.get("rightLift2");

    }

    public void LiftSystem(){

        //backup initialization code

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);

        RightLift1.setPower(0);
        RightLift2.setPower(0);
    }

    //One full revolution is 300 ticks

    public void LiftLowLevel(){

        LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift1.setTargetPosition(200);
        LeftLift2.setTargetPosition(200);
        RightLift1.setTargetPosition(200);
        RightLift2.setTargetPosition(200);

        LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift1.setPower(0.7);
        LeftLift2.setPower(0.7);
        RightLift1.setPower(0.7);
        RightLift2.setPower(0.7);

        while (LeftLift1.isBusy() && LeftLift2.isBusy() && RightLift1.isBusy() && RightLift2.isBusy())
        {

        }

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);
        RightLift1.setPower(0);
        RightLift2.setPower(0);

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LiftMiddleLevel(){

        LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift1.setTargetPosition(700);
        LeftLift2.setTargetPosition(700);
        RightLift1.setTargetPosition(700);
        RightLift2.setTargetPosition(700);

        LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift1.setPower(0.7);
        LeftLift2.setPower(0.7);
        RightLift1.setPower(0.7);
        RightLift2.setPower(0.7);

        while (LeftLift1.isBusy() && LeftLift2.isBusy() && RightLift1.isBusy() && RightLift2.isBusy())
        {

        }

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);
        RightLift1.setPower(0);
        RightLift2.setPower(0);

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void LiftHighLevel(){

        LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift1.setTargetPosition(1200);
        LeftLift2.setTargetPosition(1200);
        RightLift1.setTargetPosition(1200);
        RightLift2.setTargetPosition(1200);

        LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift1.setPower(0.7);
        LeftLift2.setPower(0.7);
        RightLift1.setPower(0.7);
        RightLift2.setPower(0.7);

        while (LeftLift1.isBusy() && LeftLift2.isBusy() && RightLift1.isBusy() && RightLift2.isBusy())
        {

        }

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);
        RightLift1.setPower(0);
        RightLift2.setPower(0);

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void LowerLowLevel(){

        LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift1.setTargetPosition(-200);
        LeftLift2.setTargetPosition(-200);
        RightLift1.setTargetPosition(-200);
        RightLift2.setTargetPosition(-200);

        LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift1.setPower(-0.7);
        LeftLift2.setPower(-0.7);
        RightLift1.setPower(-0.7);
        RightLift2.setPower(-0.7);

        while (LeftLift1.isBusy() && LeftLift2.isBusy() && RightLift1.isBusy() && RightLift2.isBusy())
        {

        }

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);
        RightLift1.setPower(0);
        RightLift2.setPower(0);

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void LowerMiddleLevel(){

        LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift1.setTargetPosition(-700);
        LeftLift2.setTargetPosition(-700);
        RightLift1.setTargetPosition(-700);
        RightLift2.setTargetPosition(-700);

        LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift1.setPower(-0.7);
        LeftLift2.setPower(-0.7);
        RightLift1.setPower(-0.7);
        RightLift2.setPower(-0.7);

        while (LeftLift1.isBusy() && LeftLift2.isBusy() && RightLift1.isBusy() && RightLift2.isBusy())
        {

        }

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);
        RightLift1.setPower(0);
        RightLift2.setPower(0);

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LowerHighLevel(){

        LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift1.setTargetPosition(-1200);
        LeftLift2.setTargetPosition(-1200);
        RightLift1.setTargetPosition(-1200);
        RightLift2.setTargetPosition(-1200);

        LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift1.setPower(-0.7);
        LeftLift2.setPower(-0.7);
        RightLift1.setPower(-0.7);
        RightLift2.setPower(-0.7);

        while (LeftLift1.isBusy() && LeftLift2.isBusy() && RightLift1.isBusy() && RightLift2.isBusy())
        {

        }

        LeftLift1.setPower(0);
        LeftLift2.setPower(0);
        RightLift1.setPower(0);
        RightLift2.setPower(0);

        LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop() {
        if (xGamepad2().dpad_up.wasPressed()){
            LiftHighLevel();
        }
        if (xGamepad2().dpad_down.wasPressed()){
            LowerHighLevel();
        }



    }

}


