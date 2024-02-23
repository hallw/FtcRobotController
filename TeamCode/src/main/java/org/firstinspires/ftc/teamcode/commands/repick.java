package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class repick implements Command {

    CrabRobot robot;
    //double duration;
    NanoClock clock;
    DriveTrain mecanumDrive;

    public repick(CrabRobot robot, DriveTrain drive) {
        this.robot= robot;
        this.mecanumDrive = drive;
        clock = NanoClock.system();
    }

    @Override
    public void start() {
        this.robot.intake.intakeState = 51;
        mecanumDrive.setDrivePower(new Pose2d(-0.05, 0, 0));
    }

    @Override
    public void update() {
        if(robot.intake.isIntakeDone == true){
            mecanumDrive.setDrivePower(new Pose2d(0.3,0,0));
        }
    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePower(new Pose2d(0,0,0));
    }

    @Override
    public boolean isCompleted() {
        return (this.robot.intake.intakeState == 0);
    }
}
