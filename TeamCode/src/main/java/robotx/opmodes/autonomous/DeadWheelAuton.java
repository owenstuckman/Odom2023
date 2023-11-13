package robotx.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import robotx.modules.ClawSystem;
import robotx.modules.LiftMotors;
import robotx.modules.MecanumDrive;
import robotx.modules.OdomSystem;
import robotx.modules.OrientationDrive;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@Autonomous(name = "DeadWheelAuton", group = "Default")

public class DeadWheelAuton extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();

    //Modules being imported
    MecanumDrive mecanumDrive;
    OrientationDrive orientationDrive;
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


        telemetry.addData(">", "Press Play to Start Op Mode");
        telemetry.update();

        int sleepTime = 1000;
        // move dead wheels down
        odomSystem.OdomSys();


        // init drive system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


       /*
       The following trajectory sequence gets us to the cone stack with the lift 400 ticks up
       There is then a second sequence that will run repeatedly to get all of the cones (a cycle)
        */

        TrajectorySequence drivetopole = drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90.00)))

                    .addTemporalMarker(0.1, () -> {
                        rawRaise(.25,10);
                        moveClawDown(50);
                     })
                .splineTo(new Vector2d(36, -15), Math.toRadians(90))

                    .addTemporalMarker(.5,() -> {
                        rawRaise(1,550);
                        moveClawUp(100);
                    })
                .splineTo(new Vector2d(27, -4), Math.toRadians(145))
                .waitSeconds(2)
                    .addTemporalMarker(3.5,() -> {
                        openClaw(100);
                    })

                .waitSeconds(30)
                .addTemporalMarker(6, () -> {

                })

                .build();

        TrajectorySequence drivetostack = drive.trajectorySequenceBuilder(drivetopole.end()).setReversed(true)
                .addTemporalMarker(0.2,() ->{
                    closeClaw(100);
                })
                .addTemporalMarker(1,() ->{
                    moveClawDown(100);
                    openClaw(100);
                })

                .splineTo(new Vector2d(62, -12), Math.toRadians(180))


                .addTemporalMarker(1,() ->{

                    // tune for first cone
                    rawLower(1,250);
                })

                .build();

        /*
        These trajectory sequences are for our cycle
         */

        // see end of drive to stack - 'home' position
        Pose2d holder = new Pose2d(62, -12, Math.toRadians(0));

        TrajectorySequence cyclepole = drive.trajectorySequenceBuilder(holder)
                .splineTo(new Vector2d(42.99, -12.25), Math.toRadians(170.54))
                .addTemporalMarker(.3,() -> {
                    //LiftTicks(1000);
                    moveClawUp(100);
                })
                .splineTo(new Vector2d(31.35, -7.56), Math.toRadians(172.04))
                .build();

        TrajectorySequence cyclestack = drive.trajectorySequenceBuilder(cyclepole.end()).setReversed(true)
                .splineTo(new Vector2d(42.99, -12.25), Math.toRadians(170.54))
                .addTemporalMarker(.1,() -> {
                    //tune
                    closeClaw(100);
                })
                .addTemporalMarker(.3,() -> {
                    //tune
                    LowerTicks(600);
                    moveClawDown(100);
                })
                .splineTo(new Vector2d(64.01,-11.9), Math.toRadians(5))
                .build();


        // starts with the first pose estimate
        drive.setPoseEstimate(drivetopole.start());

        // waits for start
        waitForStart();
        closeClaw(100);
        sleep(1000);


        // first cone driev sequence
        drive.followTrajectorySequence(drivetopole);
        drive.followTrajectorySequence(drivetostack);

        if(isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) ;
    }

    int counter = 0;
    //useless as each time we go to the stack we need lift to be at different heights for optimal speed
    public void colorStack(){
        // below 12 the cone is in the claw

        rawLower(0.5,300);

        for (int i = 0; i <300 ; i++){
            if (((DistanceSensor) liftMotors.colorSensor).getDistance(DistanceUnit.CM) < 12){
                break;
            }
            else
            {
                rawLower(0.25,10);
            }

        }
    }
    public void getconefromStack (){

        if(counter == 0){

            DriveToWall(0.25);
            sleep(100);
            closeClaw(100);

            //constant + 200 + counter * 50
            LiftTicks(400 + (counter * 50));
        }
        else {

            DriveToWall(0.25);
            sleep(100);

            // note : need to tune this initial constant
            LowerTicks((counter * 50));
            sleep(100);
            closeClaw(100);

            //constant + counter * 50
            LiftTicks((counter * 50) + 200);
        }



        counter = counter + 1;

    }


    public void resetmanipBottom(){
        // note: also will be replaced with markers
        closeClaw(50);
        moveClawDown(50);

    }

    public void dropCone(){

        // note : ignore as will replace with marker

        LiftHighLevel();
        sleep(100);
        moveClawUp(100);
        sleep(100);
        openClaw(100);
        sleep(100);
        closeClaw(100);
        sleep(100);
        moveClawDown(100);
        sleep(100);
        LowerHighLevel();

    }

    public void DriveToWall(double power){

        for (int i = 0; i < 10000; i++){

            sleep(1);
            if(!liftMotors.touch.isPressed()){
                DriveBackward(power,10);
            }
            else {
                DriveBackward(.5,100);
                break;

            }
        }
    }

    public void rawLower(double power, int time) {
        liftMotors.LeftLift1.setPower(-power);
        liftMotors.LeftLift2.setPower(-power);
        liftMotors.RightLift1.setPower(-power);
        liftMotors.RightLift2.setPower(-power);
        sleep(time);
        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);
    }
    public void rawRaise(double power, int time) {
        liftMotors.LeftLift1.setPower(power);
        liftMotors.LeftLift2.setPower(power);
        liftMotors.RightLift1.setPower(power);
        liftMotors.RightLift2.setPower(power);
        sleep(time);
        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);
    }

    public void LowerStackTicks(int ticks) {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(ticks);
        liftMotors.LeftLift2.setTargetPosition(ticks);
        liftMotors.RightLift1.setTargetPosition(-ticks);
        liftMotors.RightLift2.setTargetPosition(-ticks);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(0.7);
        liftMotors.LeftLift2.setPower(0.7);
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

    public void LiftTicks(int ticks) {


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

        while ( liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.LeftLift1.isBusy()) {

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
    public void LowerTicks(int ticks) {

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
    public void LiftHighLevel() {

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

    public void liftLift(double power, int time) {
        liftMotors.LeftLift1.setPower(power);
        liftMotors.LeftLift2.setPower(power);
        liftMotors.RightLift1.setPower(power);
        liftMotors.RightLift2.setPower(power);
        sleep(time);
        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);
    }

    public void lowerLift(double power, int time) {
        liftMotors.LeftLift1.setPower(-power);
        liftMotors.LeftLift2.setPower(-power);
        liftMotors.RightLift1.setPower(-power);
        liftMotors.RightLift2.setPower(-power);
        sleep(time);
        liftMotors.LeftLift1.setPower(0);
        liftMotors.LeftLift2.setPower(0);
        liftMotors.RightLift1.setPower(0);
        liftMotors.RightLift2.setPower(0);
    }

    public void DriveForward(double power, int time) {
        mecanumDrive.frontLeft.setPower(-power);  //top left when rev is down and ducky wheel is right
        mecanumDrive.frontRight.setPower(power);  //bottom left
        mecanumDrive.backLeft.setPower(power);    //top right
        mecanumDrive.backRight.setPower(-power);  // bottom right
        sleep(time);
        mecanumDrive.frontLeft.setPower(0);
        mecanumDrive.frontRight.setPower(0);
        mecanumDrive.backLeft.setPower(0);
        mecanumDrive.backRight.setPower(0);
    }

    public void DriveBackward(double power, int time) {
        mecanumDrive.frontLeft.setPower(power);
        mecanumDrive.frontRight.setPower(-power);
        mecanumDrive.backLeft.setPower(-power);
        mecanumDrive.backRight.setPower(power);
        sleep(time);
        mecanumDrive.frontLeft.setPower(0);
        mecanumDrive.frontRight.setPower(0);
        mecanumDrive.backLeft.setPower(0);
        mecanumDrive.backRight.setPower(0);
    }

    public void StrafeRight(double power, int time) {
        mecanumDrive.frontLeft.setPower(-power);
        mecanumDrive.frontRight.setPower(-power);
        mecanumDrive.backLeft.setPower(-power);
        mecanumDrive.backRight.setPower(-power);
        sleep(time);
        mecanumDrive.frontLeft.setPower(0);
        mecanumDrive.frontRight.setPower(0);
        mecanumDrive.backLeft.setPower(0);
        mecanumDrive.backRight.setPower(0);
    }

    public void StrafeLeft(double power, int time) {
        mecanumDrive.frontLeft.setPower(power);
        mecanumDrive.frontRight.setPower(power);
        mecanumDrive.backLeft.setPower(power);
        mecanumDrive.backRight.setPower(power);
        sleep(time);
        mecanumDrive.frontLeft.setPower(0);
        mecanumDrive.frontRight.setPower(0);
        mecanumDrive.backLeft.setPower(0);
        mecanumDrive.backRight.setPower(0);
    }

    public void TurnLeft(double power, int time) {
        mecanumDrive.frontLeft.setPower(power);
        mecanumDrive.frontRight.setPower(-power);
        mecanumDrive.backLeft.setPower(power);
        mecanumDrive.backRight.setPower(-power);
        sleep(time);
        mecanumDrive.frontLeft.setPower(0);
        mecanumDrive.frontRight.setPower(0);
        mecanumDrive.backLeft.setPower(0);
        mecanumDrive.backRight.setPower(0);
    }

    public void TurnRight(double power, int time) {
        mecanumDrive.frontLeft.setPower(-power);
        mecanumDrive.frontRight.setPower(power);
        mecanumDrive.backLeft.setPower(-power);
        mecanumDrive.backRight.setPower(power);
        sleep(time);
        mecanumDrive.frontLeft.setPower(0);
        mecanumDrive.frontRight.setPower(0);
        mecanumDrive.backLeft.setPower(0);
        mecanumDrive.backRight.setPower(0);
    }

}


