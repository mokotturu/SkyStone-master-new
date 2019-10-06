/**
 * Autonomous on the blue building site side.
 *
 * NOTES:
 * - Transfer the rotate() code to Autonomous for Depot side.
 * - Update the code once we have the ultrasonic sensors.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.openftc.revextensions2.*;

@Autonomous(name="Autonomous Building Site", group="Autonomous")
public class AutonomousBuildingSite extends LinearOpMode {
    DcMotor                 leftFront, leftBack, rightFront, rightBack, intakeMotor;
    Servo                   foundationServo;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    PIDController           pidRotate, pidDrive;
    double                  globalAngle, power = .30, correction, rotation;
    double                  rotationPower = 0.5;
    double                  movePower = 0.7;
    static final double     COUNTS_PER_MOTOR_REV  = 537.6;
    static final double     DRIVE_GEAR_REDUCTION  = 1.0;
    static final double     WHELL_DIAMETER_INCHES = 3.937;
    static final double     COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHELL_DIAMETER_INCHES * 3.1415);
    ExpansionHubMotor intakeMotorRE2;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        intakeMotor = hardwareMap.dcMotor.get("intake motor");

        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("left front");;

        foundationServo = hardwareMap.servo.get("foundationServo");

        foundationServo.setPosition(0);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Change from encoders to ultrasonic sensor when available

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        if (opModeIsActive()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1. imu heading", lastAngles.firstAngle);
            telemetry.addData("2. global heading", globalAngle);
            telemetry.addData("3. correction", correction);
            telemetry.update();

            /*leftFront.setPower(power - correction);
            leftBack.setPower(power - correction);
            rightFront.setPower(power + correction);
            rightBack.setPower(power + correction);*/

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            // Moving to the foundation, pulling it, and then moving to the line
            /*move(5, movePower/2, false);
            foundationServo.setPosition(0.5);
            strafe(24, movePower, true);
            move(25, movePower, false);
            Thread.sleep(300);
            move(1, movePower/3, false);
            foundationServo.setPosition(0);
            Thread.sleep(300);
            move(27, movePower/1.7, true);  // change this to "when hitting the wall" once we get an ultrasonic sensor
            foundationServo.setPosition(0.5);
            Thread.sleep(15000);
            strafe(20, movePower, false);
            move(5, movePower, false);
            foundationServo.setPosition(0);
            strafe(28, movePower, false);*/

            intakeMotor.setPower(1);

            telemetry.addData("Intake Motor current", intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();
        }
    }

    // set direction to true if strafing right, false if strafing left
    private void strafe(int distance, double power, boolean direction) {
        if(distance == 0) return;

        telemetry.addData("strafing direction", direction);
        telemetry.update();

        int targetPos = (int)(distance * COUNTS_PER_INCH);

        double myPower;
        int strafeDistance;

        if(direction) {
            myPower = power;
            strafeDistance = targetPos;
        } else {
            myPower = -power;
            strafeDistance = -targetPos;
        }

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(strafeDistance);
        leftBack.setTargetPosition(-strafeDistance);
        rightFront.setTargetPosition(-strafeDistance);
        rightBack.setTargetPosition(strafeDistance);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(myPower);
        leftBack.setPower(-myPower);
        rightFront.setPower(-myPower);
        rightBack.setPower(myPower);

        while(leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) { }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void move(int distance, double power, boolean direction) {
        if(distance == 0) return;

        telemetry.addData("moving direction", direction);
        telemetry.update();

        int targetPos = (int)(distance * COUNTS_PER_INCH);

        double myPower;
        int moveDistance;

        if(direction) {
            myPower = power;
            moveDistance = targetPos;
        } else {
            myPower = -power;
            moveDistance = -targetPos;
        }

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(moveDistance);
        leftBack.setTargetPosition(moveDistance);
        rightFront.setTargetPosition(moveDistance);
        rightBack.setTargetPosition(moveDistance);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(myPower);
        leftBack.setPower(myPower);
        rightFront.setPower(myPower);
        rightBack.setPower(myPower);

        while(leftFront.isBusy() && leftBack.isBusy() && rightBack.isBusy() && rightFront.isBusy()) { }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction *= gain;

        return correction;
    }
}