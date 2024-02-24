package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@TeleOp
public class TestDistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        Utilities.getSharedUtility().initialize(this);
        DistanceSensor DistanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        DistanceSensor DistanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        waitForStart();

        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();


        while (!isStopRequested()) {
            telemetry.update();
            telemetry.addData("DistR: ", DistanceR.getDistance(DistanceUnit.CM));
            telemetry.addData("DistL: ", DistanceL.getDistance(DistanceUnit.CM));
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());
            double currentTime = clock.seconds();
            telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
        }
    }
}