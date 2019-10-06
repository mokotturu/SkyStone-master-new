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

@Autonomous(name="IntakeTesting", group="Autonomous")
public class IntakeTesting extends LinearOpMode {
    DcMotor                 leftFront, leftBack, rightFront, rightBack, intakeMotor;
    ExpansionHubMotor intakeMotorRE2;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.dcMotor.get("intake motor");

        intakeMotorRE2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake motor");;

        while (!isStopRequested()) {
            sleep(50);
            idle();
        }

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        if (opModeIsActive()) {
            while (!isStopRequested()) {
                intakeMotor.setPower(1);
                telemetry.addData("Intake Motor current", intakeMotorRE2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                telemetry.update();
            }
        }
    }
}