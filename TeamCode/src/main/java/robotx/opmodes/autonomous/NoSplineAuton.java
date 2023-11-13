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


@Autonomous(name = "NoSplineAuton", group = "Default")

public class NoSplineAuton extends LinearOpMode {
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

        liftMotors.LeftLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotors.LeftLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotors.RightLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotors.RightLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odomSystem = new OdomSystem(this);
        odomSystem.init();
        odomSystem.start();
        // move dead wheels down
        odomSystem.OdomSys();


        telemetry.addData(">", "Press Play to Start Op Mode");
        telemetry.update();

        int sleepTime = 1000;

        // init drive system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // all positions we move to here
        Pose2d startingPosition = new Pose2d(36, -60, Math.toRadians(90.00));
        Pose2d polePosition = new Pose2d(26, -7, Math.toRadians(135));
        Pose2d safePosition = new Pose2d(36, -12, Math.toRadians(180));
        Pose2d stackPosition = new Pose2d(62, -12, Math.toRadians(180));

        // all trajectories here
        Trajectory safeToPole1 = drive.trajectoryBuilder(startingPosition)
                .lineToLinearHeading(safePosition.plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .build();
        Trajectory safeToPole = drive.trajectoryBuilder(polePosition)
                .lineToLinearHeading(safePosition)
                .build();
        Trajectory toPole = drive.trajectoryBuilder(safePosition)
                .lineToLinearHeading(polePosition)
                .build();
        Trajectory toConeStack = drive.trajectoryBuilder(safePosition)
                .lineToLinearHeading(stackPosition,
                        // limits velocity when going to stack so it doesnt crash into wall
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory goBackToPole = drive.trajectoryBuilder(stackPosition)
                .lineToLinearHeading(polePosition)
                .build();


        // inits position into where it's actually starting
        drive.setPoseEstimate(startingPosition);

        openClaw(100);

        waitForStart();

        closeClaw(100);
        sleep(sleepTime);
        rawLift(1, 15);
        sleep(sleepTime);

        // *
        // DRIVE STARTS HERE
        // *
        drive.followTrajectory(safeToPole1);
        sleep(sleepTime);
        drive.followTrajectory(toPole);
        sleep(sleepTime);
        dropCone(sleepTime);
        sleep(sleepTime);
        drive.followTrajectory(safeToPole);

        // repeats for however many cones we want
        for (int cones = 2; cones > 0; cones--) {
            drive.followTrajectory(toConeStack);
            openClaw(100);
            sleep(sleepTime);
            drive.followTrajectory(goBackToPole);
            sleep(sleepTime);
            dropCone(sleepTime);
        }

        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void dropCone(int sleepTime) {
        // *TO DO* add liftTicks so lift is consistent w voltage
        // up
        closeClaw(100);
        sleep(sleepTime);
        rawLift(1,500);
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
