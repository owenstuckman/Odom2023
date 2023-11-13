package robotx.OldModules;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import robotx.libraries.AutonomousSystem;

/**
 * Created by Kush Dalal on 10/31/2018
 */
public class TwoWheelAutonMU extends AutonomousSystem {

    int gyr;
    BNO055IMU gyroSensor;
    Orientation lastAngles = new Orientation();
    double globalAngle;


    // Be sure to assign these before use.
    public DcMotor leftMotor;
    public DcMotor rightMotor;


    public TwoWheelAutonMU(OpMode op) {
        super(op);
    }

    public void init() {
        //gyroSensor = (BNO055IMU) opMode.hardwareMap.gyroSensor.get("gyroSensor");
        //resetAngle();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of ty0pe "AdaFruit IMU",
        // and named "imu".
        gyroSensor = opMode.hardwareMap.get(BNO055IMU.class, "gyroSensor");
        gyroSensor.initialize(parameters);

        opMode.telemetry.addData("heading: ", getHeadingAngle());
        opMode.telemetry.update();


    }

    // Return the current heading angle of the robot.
    // This should not loop around at 360, and values should increase past 360.

	/*public void resetAngle() {
		lastAngles = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		globalAngle = 0;
	}*/

    public int getHeadingAngle() {

        Orientation angles = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        int finalAngle = (int) globalAngle;

        lastAngles = angles;

        return finalAngle;

    }

    // Return the current distance the left side has traveled, in encoder ticks.
    public int getLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    // Return the current distance the right side has traveled, in encoder ticks.
    public int getRightTicks() {
        return rightMotor.getCurrentPosition();
    }

    // Input encoder ticks and return centimeters.
    public double ticksToCentimeters(int encoderTicks) {
        return encoderTicks * ((42.25 / 3600.0) * 2.54);
    }


}