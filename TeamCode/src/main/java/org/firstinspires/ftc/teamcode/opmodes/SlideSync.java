package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.Utilities;

import java.util.ArrayList;

@Config
@TeleOp(name="Test: SlideSync", group="Test")
public class SlideSync extends LinearOpMode {

    public static int velocity = 500;
    public static double power = 0.0;
    double leftMinUpPower = 0.08;
    double rightMinUpPower = 0.11;
    double leftMinDownPower = 0.04;
    double rightMinDownPower = 0.04;
    public static int dir = 1;
    public static int upTargetPos = 2000;

    @Override
    public void runOpMode() throws InterruptedException {

        Utilities.getSharedUtility().initialize(this);
        SmartGamepad smartGamepad1 = new SmartGamepad(gamepad1);
        SmartGamepad smartGamepad2 = new SmartGamepad(gamepad2);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        DcMotorEx motorL, motorR;
        motorL  = hardwareMap.get(DcMotorEx.class, "slideLt");
        motorR  = hardwareMap.get(DcMotorEx.class, "slideRt");

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL.setDirection(DcMotorEx.Direction.REVERSE);
//        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            //Log.v("slideTestLog", "nothing");

            if (smartGamepad1.a_pressed()) {
                Log.v("slideTestLog", "to up");
                motorL.setTargetPosition(upTargetPos);
                motorR.setTargetPosition(upTargetPos);
                motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorL.setVelocity(velocity);
                motorR.setVelocity(velocity);
            } else if (smartGamepad1.b_pressed()) {
                Log.v("slideTestLog", "to down");
                motorL.setTargetPosition(0);
                motorR.setTargetPosition(0);
                motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorL.setVelocity(-velocity);
                motorR.setVelocity(-velocity);
            }
            motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorL.setPower(dir*power);
            motorR.setPower(dir*power);
            if(smartGamepad1.dpad_up_pressed()){
                Log.v("slideTestLog", "power up " + power);
                power+=0.01;
            }else if(smartGamepad1.dpad_down_pressed()){
                Log.v("slideTestLog", "power down " + power);
                power = power - 0.01;
            } else if(smartGamepad1.dpad_right_pressed()){
                dir = - dir;
            }
            smartGamepad1.update(packet);
            smartGamepad2.update(packet);
            packet.put("L_encoder", motorL.getCurrentPosition() );
            packet.put("R_encoder", motorR.getCurrentPosition() );
            packet.put("power", power );
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
