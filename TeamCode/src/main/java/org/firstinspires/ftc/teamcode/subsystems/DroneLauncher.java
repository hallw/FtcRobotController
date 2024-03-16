package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
@Config
public class DroneLauncher implements Subsystem {
    public Servo droneTrigger;
    Telemetry telemetry;
    public static double holdPos = 0.45;
    public static double releasePos = 0.65;//0.1

    public DroneLauncher(Robot robot, Telemetry telemetry) {
        this.telemetry = telemetry;
        droneTrigger = robot.getServo("droneTrigger");
        hold();
    }
    public void hold(){
        droneTrigger.setPosition(holdPos);
    }
    public void release(){
        droneTrigger.setPosition(releasePos);
    }
    public void setPos(double pos) {
        droneTrigger.setPosition(pos);
    }

    public void update(TelemetryPacket packet){

    }
}
