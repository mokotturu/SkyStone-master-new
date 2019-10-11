/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "DistanceSensorTest", group = "Autonomous")
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor leftDistance, rightDistance;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        // leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor leftDistanceToF = (Rev2mDistanceSensor) leftDistance;
        Rev2mDistanceSensor rightDistanceToF = (Rev2mDistanceSensor) rightDistance;

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        // waitForStart();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start command...");
            telemetry.update();
        }

        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            /*telemetry.addData("deviceName", leftDistance.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", leftDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", leftDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", leftDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", leftDistance.getDistance(DistanceUnit.INCH)));*/

            /*telemetry.addData("deviceName", rightDistance.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", rightDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", rightDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", rightDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", rightDistance.getDistance(DistanceUnit.INCH)));

            telemetry.update();*/

            telemetry.addData("Color Distance Sensor: Distance (in)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

}