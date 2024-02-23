package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.commands.autoOutPrep;
import org.firstinspires.ftc.teamcode.commands.autoOutDump;
import org.firstinspires.ftc.teamcode.commands.repick;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class ZAlignTest extends LinearOpMode {
    public static double drivePwr = 0.2;
    public static double hCoeff = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        // Commands
        //Servo init code here
        alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,10, telemetry);
        autoOutPrep outPrep = new autoOutPrep(robot,false);
        autoOutDump outDump = new autoOutDump(robot, drivetrain);
        repick repick = new repick(robot,drivetrain);

        // Start
        waitForStart();
        if (isStopRequested()) return;

        //Log.v("Align", "staring command" );
        //robot.runCommand(outPrep);
        //robot.runCommand(alignCmd);
        robot.runCommand(repick);
        //robot.runCommand(outDump);

        Log.v("Align", "end command" );
    }
}
