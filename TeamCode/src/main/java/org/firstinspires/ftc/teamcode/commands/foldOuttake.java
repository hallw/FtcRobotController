package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

//autoOut needs to do 3 outtake actions:
// 1:             robot.intake.setPower(0);
//                robot.intake.toBasePos();
//                robot.outtake.prepOuttake();
// 2:             robot.outtake.dropPixelPos();
// 3:             robot.outtake.toIntakePos();
public class foldOuttake implements Command {

    CrabRobot robot;
    DriveTrain mecanumDrive;
    //double duration;
    NanoClock clock;
    long time = System.currentTimeMillis();
    int state = 0;

    public foldOuttake(CrabRobot robot, DriveTrain drive) {
        this.robot= robot;
        this.mecanumDrive = drive;
        clock = NanoClock.system();
    }

    @Override
    public void start() { // gamepad2.a
        robot.outtake.dumperToIntakePosAuto();
        time = System.currentTimeMillis();
        state = 1;
        //Log.v("autoOut", "dropPixelPosAuto()");
    }

    @Override
    public void update() {
        if (state == 1) {
            if (System.currentTimeMillis() - time > 100) { // dumper reset time
                robot.outtake.toIntakePosAuto(); // back to intake pos
                state = 2;
                time = System.currentTimeMillis();
                Log.v("autoOutDump", "-> toIntakePosAuto()");
            }
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        if (this.state == 2
                && robot.outtake.swingState == 0
                && robot.outtake.liftState == 0
                && (System.currentTimeMillis() - time > 400)) {
            state = 0;
            return (true);
        } else {
            return (false);
        }
    }
}
