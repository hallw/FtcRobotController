package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import android.util.Log;

//autoOut needs to do 3 outtake actions:
// 1:             robot.intake.setPower(0);
//                robot.intake.toBasePos();
//                robot.outtake.prepOuttake();
// 2:             robot.outtake.dropPixelPos();
// 3:             robot.outtake.toIntakePos();
public class autoOutPrep implements Command {

    CrabRobot robot;
    //double duration;
    NanoClock clock;
    long time = System.currentTimeMillis();
    int state = 0;

    public autoOutPrep(CrabRobot robot) {
        this.robot= robot;
        clock = NanoClock.system();
    }

    @Override
    public void start() { // gamepad2.a
        this.robot.intake.setPower(0);
        this.robot.intake.toBasePosYield();
        this.robot.outtake.prepOuttakeAuto();
        time = System.currentTimeMillis();
        state = 1;
        Log.v("autoOut", "start()");
    }

    @Override
    public void update() {
        if (state == 1) { // wait for lift state to be != 0
            if (System.currentTimeMillis() - time > 100) {
                state = 2;
                Log.v("autoOut", "state 1 -> 2");
            }
        } else if (state == 2) {//wait for lift reach state 0
            if (robot.outtake.liftState == 0 && robot.outtake.swingState == 0) { //gamepad2.a done,
                time = System.currentTimeMillis();
                state = 3;
                Log.v("autoOut", "state 2 -> 3");
            }
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        if (this.state == 3 && (System.currentTimeMillis() - time > 400)) {
            state = 0;
            Log.v("autoOut", "Done");
            return (true);
        } else {
            return (false);
        }
    }
}
