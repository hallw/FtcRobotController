package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoOutPrep;
import org.firstinspires.ftc.teamcode.commands.autoOutDump;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Config
@Autonomous
public class outCmdTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        // Commands
        //Servo init code here
        robot.intake.toBasePos();
        //robot.outtake.toIntakePos();
        autoOutPrep outCmd = new autoOutPrep(robot,false);
        autoOutDump dumpCmd = new autoOutDump(robot, drivetrain);

        // Start
        waitForStart();
        if (isStopRequested()) return;
        //Log.v("AUTODEBUG", "0: start");

        robot.runCommand(outCmd);
        robot.runCommand(dumpCmd);
    }
}
