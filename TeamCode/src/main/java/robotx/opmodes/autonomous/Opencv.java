package robotx.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import robotx.modules.ClawSystem;
import robotx.modules.LiftMotors;
import robotx.modules.MecanumDrive;
import robotx.modules.OdomSystem;
import robotx.modules.OrientationDrive;

@TeleOp(name = "OpenCv", group = "Default")
public class Opencv extends LinearOpMode {

    OpenCvWebcam phoneCam;
    SkystoneDeterminationPipeline pipeline;
    MecanumDrive mecanumDrive;
    OrientationDrive orientationDrive;
    LiftMotors liftMotors;
    ClawSystem clawSystem;
    OdomSystem odomSystem;

    @Override
    public void runOpMode() {

        // Setup, initialize, import modules

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        orientationDrive = new OrientationDrive(this);
        orientationDrive.init();

        mecanumDrive = new MecanumDrive(this);
        mecanumDrive.init();

        liftMotors = new LiftMotors(this);
        liftMotors.init();

        clawSystem = new ClawSystem(this);
        clawSystem.init();

        odomSystem = new OdomSystem(this);
        odomSystem.init();

        mecanumDrive.start();
        orientationDrive.start();
        liftMotors.start();
        clawSystem.start();

        mecanumDrive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int sleepTime = 1000;
        String coneColor = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive()) {

            //loop for 5 seconds - if it gets stuck for some really weird reason it will go into two just to be safe
             moveClawDown(100);
             sleep(sleepTime);
             closeClaw(100);
             sleep(sleepTime);
             rawLift(1, 25);
             sleep(sleepTime);
             sleep(2000);

            for (int i = 0; i<5000; i++) {

                sleep(5);
                telemetry.addData("AnalysisCb", pipeline.getAnalysis1());
                telemetry.addData("AnalysisCb", pipeline.getAnalysis2());
                telemetry.update();


                //note: need to tune minus values
                //note: need to pick colors
                //note: need movements

                //white
                if (pipeline.getAnalysis1() == pipeline.getAnalysis2() || pipeline.getAnalysis1() -1 == pipeline.getAnalysis2() || pipeline.getAnalysis1() == pipeline.getAnalysis2() -1 ) {

                    coneColor = "white";
                    break;
                //cyan
                } else if (pipeline.getAnalysis1() < pipeline.getAnalysis2() - 30) {

                    coneColor = "cyan";
                    break;

                // yellow
                } else if (pipeline.getAnalysis1() - 30 > pipeline.getAnalysis2()) {

                    coneColor = "yellow";
                    break;

                }
                else if(i == 4000){

                    coneColor = "white";
                    break;

                }


            }

            //one
            if (coneColor.equals("yellow")){

                // movement
                DriveForward(1,500);
                sleep(sleepTime);
                StrafeLeft(1,600);
                sleep(sleepTime);
                sleep(30000);
            }
            //two
            if (coneColor.equals("white")){

                // movement
                DriveForward(.7,1200);
                sleep(30000);
            }
            //three
            if (coneColor.equals("cyan")){

                // movement
                DriveForward(1,500);
                sleep(sleepTime);
                StrafeRight(1,700);
                sleep(sleepTime);
                sleep(30000);
            }
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        /*
         * Some color constants
         */

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final int duckThreshold = 100;

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(163, 96);

        static final int REGION_WIDTH = 10;
        static final int REGION_HEIGHT = 25;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region1_Cr;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat Cr = new Mat();

        int avg1;
        int avg2;

        int avgCb;
        int avgCr;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         * -> also added the Cr Channel
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }
        void inputToCr(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 2);
        }


        public void init(Mat firstFrame) {

            inputToCb(firstFrame);
            inputToCr(firstFrame);

            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            inputToCr(input);

            avgCb = (int) Core.mean(region1_Cb).val[0];
            avgCr = (int) Core.mean(region1_Cr).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis1() {
            avg1 = avgCb;
            return avgCb;
        }
        public int getAnalysis2(){
            avg2 = avgCr;
            return avgCr;
        }
    }

    //Controls
    int timeCheck = 1;



    public void odomsUp(int time){
        odomSystem.odom1.setPosition(0.46);
        odomSystem.odom1.setPosition(0.11);
        odomSystem.odom1.setPosition(0.46);
        sleep(time);
    }
    public void odomsDown(int time){
        odomSystem.odom1.setPosition(0.1);
        odomSystem.odom1.setPosition(0.77);
        odomSystem.odom1.setPosition(0.21);
        sleep(time);
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

    public void GearLockOpen(){

        liftMotors.gearServo.setPosition(.88);
        sleep(20);

    }

    public void GearLockClose(){

        liftMotors.gearServo.setPosition(.69);
        sleep(20);

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

    /*public void LiftLowLevel(){

            liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftMotors.LeftLift1.setTargetPosition(200);
            liftMotors.LeftLift2.setTargetPosition(200);
            liftMotors.RightLift1.setTargetPosition(200);
            liftMotors.RightLift2.setTargetPosition(200);

            liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotors.LeftLift1.setPower(0.7);
            liftMotors.LeftLift2.setPower(0.7);
            liftMotors.RightLift1.setPower(0.7);
            liftMotors.RightLift2.setPower(0.7);

            while (liftMotors.LeftLift1.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy())
            {

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

    public void LowerLowLevel(){

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(-200);
        liftMotors.LeftLift2.setTargetPosition(-200);
        liftMotors.RightLift1.setTargetPosition(-200);
        liftMotors.RightLift2.setTargetPosition(-200);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(-0.7);
        liftMotors.LeftLift2.setPower(-0.7);
        liftMotors.RightLift1.setPower(-0.7);
        liftMotors.RightLift2.setPower(-0.7);

        while (liftMotors.LeftLift1.isBusy() && liftMotors.LeftLift2.isBusy() && liftMotors.RightLift1.isBusy() && liftMotors.RightLift2.isBusy())
        {

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
*/
    public void LiftMidLevel() {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(290);
        liftMotors.LeftLift2.setTargetPosition(290);
        liftMotors.RightLift1.setTargetPosition(290);
        liftMotors.RightLift2.setTargetPosition(290);

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotors.LeftLift1.setPower(0.7);
        liftMotors.LeftLift2.setPower(0.7);
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

    public void LowerMidLevel() {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(-290);
        liftMotors.LeftLift2.setTargetPosition(-290);
        liftMotors.RightLift1.setTargetPosition(-290);
        liftMotors.RightLift2.setTargetPosition(-290);

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

    public void LiftHighLevel() {

        liftMotors.LeftLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.LeftLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors.RightLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotors.LeftLift1.setTargetPosition(1200);
        liftMotors.LeftLift2.setTargetPosition(1200);
        liftMotors.RightLift1.setTargetPosition(1200);
        liftMotors.RightLift2.setTargetPosition(1200);

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

        liftMotors.LeftLift1.setTargetPosition(-420);
        liftMotors.LeftLift2.setTargetPosition(-420);
        liftMotors.RightLift1.setTargetPosition(-420);
        liftMotors.RightLift2.setTargetPosition(-420);

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
        clawSystem.leftArm.setPosition(0.35);
        clawSystem.rightArm.setPosition(0.706);
        sleep(time);

    }

    public void moveClawUp(int time) {
        clawSystem.leftArm.setPosition(0.616);
        clawSystem.rightArm.setPosition(0.436);
        sleep(time);

    }

    public void openClaw(int time) {
        clawSystem.claw.setPosition(0.3);
        sleep(time);

    }

    public void closeClaw(int time) {
        clawSystem.claw.setPosition(0.6);
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
        mecanumDrive.frontRight.setPower(power); //bottom left
        mecanumDrive.backLeft.setPower(power);   //top right
        mecanumDrive.backRight.setPower(-power); // bottom right
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
