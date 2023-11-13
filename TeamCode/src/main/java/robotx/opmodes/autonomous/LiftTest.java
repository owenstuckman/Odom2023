package robotx.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import robotx.modules.ClawSystem;
import robotx.modules.LiftMotors;
import robotx.modules.OdomSystem;

import org.firstinspires.ftc.teamcode.drive.*;


@Autonomous(name = "LiftTest", group = "Default")

public class LiftTest extends LinearOpMode {
    ClawSystem clawSystem;
    LiftMotors liftMotors;
    OdomSystem odomSystem;

    @Override
    public void runOpMode() {

        //Text at bottom of phone
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clawSystem = new ClawSystem(this);
        clawSystem.init();

        liftMotors = new LiftMotors(this);
        liftMotors.init();


        odomSystem = new OdomSystem(this);
        odomSystem.init();
        odomSystem.start();
        // move dead wheels down
        odomSystem.OdomSys();


        telemetry.addData(">", "Press Play to Start Op Mode");
        telemetry.update();

        int sleepTime = 1000;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        openClaw(100);

        waitForStart();

        dropCone(sleepTime);

        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void dropCone(int sleepTime) {
        // *TO DO* add liftTicks so lift is consistent w voltage
        // up
        closeClaw(100);
        sleep(sleepTime);
        rawLift(1,475);
        sleep(sleepTime);
        moveClawUp(50);
        sleep(sleepTime);
        openClaw(50);
        sleep(sleepTime);
        closeClaw(50);
        sleep(sleepTime);
        //down
        moveClawDown(50);
        sleep(sleepTime);
        rawLower(1,400);
        sleep(sleepTime);
    }


    public void rawLift(double power, int time) {
        liftMotors.LeftLift1.setPower(power);
        liftMotors.LeftLift2.setPower(power);
        liftMotors.RightLift1.setPower(-power);
        liftMotors.RightLift2.setPower(-power);
        sleep(time);
        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);
    }

    public void rawLower(double power, int time) {
        liftMotors.LeftLift1.setPower(-power);
        liftMotors.LeftLift2.setPower(-power);
        liftMotors.RightLift1.setPower(power);
        liftMotors.RightLift2.setPower(power);
        sleep(time);
        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);
    }



    public void liftTicks(int ticks) {


        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.RightLift1.setTargetPosition(ticks);
        liftMotors.RightLift2.setTargetPosition(ticks);
        liftMotors.LeftLift1.setTargetPosition(-ticks);
        liftMotors.LeftLift2.setTargetPosition(-ticks);


        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(1);
        liftMotors.LeftLift2.setPower(1);
        liftMotors.RightLift1.setPower(1);
        liftMotors.RightLift2.setPower(1);

        while (liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.LeftLift1.isBusy()) {

        }

        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lowerTicks(int ticks) {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(-ticks);
        liftMotors.LeftLift2.setTargetPosition(-ticks);
        liftMotors.RightLift1.setTargetPosition(ticks);
        liftMotors.RightLift2.setTargetPosition(ticks);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(-0.7);
        liftMotors.LeftLift2.setPower(-0.7);
        liftMotors.RightLift1.setPower(0.7);
        liftMotors.RightLift2.setPower(0.7);

        while (liftMotors.LeftLift1.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy()) {

        }

        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftHighLevel() {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(1400);
        liftMotors.LeftLift2.setTargetPosition(1400);
        liftMotors.RightLift1.setTargetPosition(1400);
        liftMotors.RightLift2.setTargetPosition(1400);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(1);
        liftMotors.LeftLift2.setPower(1);
        liftMotors.RightLift1.setPower(1);
        liftMotors.RightLift2.setPower(1);

        while (liftMotors.LeftLift1.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy()) {

        }

        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LowerHighLevel() {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(-1400);
        liftMotors.LeftLift2.setTargetPosition(-1400);
        liftMotors.RightLift1.setTargetPosition(-1400);
        liftMotors.RightLift2.setTargetPosition(-1400);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(-0.7);
        liftMotors.LeftLift2.setPower(-0.7);
        liftMotors.RightLift1.setPower(-0.7);
        liftMotors.RightLift2.setPower(-0.7);

        while (liftMotors.LeftLift1.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy()) {

        }

        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveClawDown(int time) {
        clawSystem.leftArm.setPosition(0.36);
        clawSystem.rightArm.setPosition(0.696);
        sleep(time);

    }

    public void moveClawUp(int time) {
        clawSystem.leftArm.setPosition(0.62);
        clawSystem.rightArm.setPosition(0.42);
        sleep(time);

    }

    public void openClaw(int time) {
        clawSystem.claw.setPosition(0);
        sleep(time);

    }

    public void closeClaw(int time) {
        clawSystem.claw.setPosition(0.2);
        sleep(time);

    }
}
