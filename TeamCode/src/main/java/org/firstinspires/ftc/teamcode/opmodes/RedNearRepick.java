package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.commands.autoOutDump;
import org.firstinspires.ftc.teamcode.commands.autoOutPrep;
import org.firstinspires.ftc.teamcode.commands.backAfterDump;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.commands.foldOuttake;
import org.firstinspires.ftc.teamcode.commands.repick;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class RedNearRepick extends LinearOpMode {
    public static boolean parkCenter = false;
    public static boolean IS_RED = true;
    public static boolean ALIGN_RIGHT = true;
    public static double POS1_SPL1_X = 24;
    public static double POS1_SPL1_Y = -4;
    public static double POS1_DUMP_X = 31;
    public static double POS2_DUMP_X = 25;
    public static double POS3_DUMP_X = 18;
    public static double DUMP_Y = -28;

    public static double POS2_SPL1_X = 36.5;
    public static double POS2_SPL1_Y = -20;
    public static double POS3_SPL1_X = 25;
    public static double POS3_SPL1_Y = -24;
    public static double FACE_BACKDROP_HEADERING = Math.toRadians(-90);
    public static double PARK_CENTER_X = 50;
    public static double PARK_CORNER_X = 0;
    public static double REPICK_X = 48.5;
    public static double REPICK_Y = 60;
    public static double PARK_STRAFE_MIDDLE_TO_CENTER = 28;
    public static double TAG_DIST = 6;
    public static double PARK_FORWARD = 11.5;
    public static double drivePwr = 0.2;
    public static double hCoeff = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        RobotVision rvis = new RobotVision(ALIGN_RIGHT);

        // general variable
        int elementPos;

        // Commands
        alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,10, telemetry);
        //Servo init code here
        robot.intake.toBasePos();
        robot.outtake.toIntakePos();
        dropIntakePreload dropIntakePreload = new dropIntakePreload(robot);
        autoOutPrep outPrep = new autoOutPrep(robot,false);
        autoOutDump outDump = new autoOutDump(robot, drivetrain);
        backAfterDump dumpNBack = new backAfterDump(robot,drivetrain);
        foldOuttake fold = new foldOuttake(robot,drivetrain);
        repick repick = new repick(robot,drivetrain);

        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        // Start
        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;
        //Log.v("AUTODEBUG", "0: start");
        elementPos = rvis.getTeamPropOrientation(IS_RED, ALIGN_RIGHT);
        //Log.v("AUTODEBUG", "1: elementPos = %0d");
        telemetry.addData("Element pos", elementPos);
        telemetry.addData("Is parking center?: ", parkCenter);
        telemetry.update();


        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS1_SPL1_X+5, POS1_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(POS1_SPL1_X+5, POS1_SPL1_Y+2))
                            .addTemporalMarker(1.0,()->robot.runCommand(dropIntakePreload))
                            .build()
            ));
            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(POS1_DUMP_X, DUMP_Y))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);
            //Log.v("AUTODEBUG", "10: dump done");

            // Repick
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, DUMP_Y+10))
                            .addTemporalMarker(0.1,()->robot.runCommand(fold))
                            .lineTo(new Vector2d(REPICK_X, REPICK_Y))
                            .build()
            ));
            robot.runCommand(repick);
            // goto pos 3 and dump there
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, DUMP_Y+15)) // goto pos 3
                            .addTemporalMarker(2.0,()->robot.runCommand(outPrep))
                            .splineTo(new Vector2d(POS3_DUMP_X,DUMP_Y-5),FACE_BACKDROP_HEADERING)
                            .build()
            ));
            alignBackdrop alignCmd2 = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
            robot.runCommand(alignCmd2);
            robot.runCommand(outDump);

        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS2_SPL1_X, POS2_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .addTemporalMarker(2.0,()->robot.runCommands(dropIntakePreload))
                            .build()
            ));
            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(POS2_DUMP_X, DUMP_Y))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);
            // Repick
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, DUMP_Y+10))
                            .addTemporalMarker(0.1,()->robot.runCommand(fold))
                            .lineTo(new Vector2d(REPICK_X, REPICK_Y))
                            .build()
            ));
            robot.runCommand(repick);
            // goto pos 1 and dump there
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, DUMP_Y+15)) // goto pos 1
                            .addTemporalMarker(2.0,()->robot.runCommand(outPrep))
                            .splineTo(new Vector2d(POS1_DUMP_X,DUMP_Y-5),FACE_BACKDROP_HEADERING)
                            .build()
            ));
            alignBackdrop alignCmd2 = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
            robot.runCommand(alignCmd2);
            robot.runCommand(outDump);

        } else {// right
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS3_SPL1_X, POS3_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .addTemporalMarker(2.0,()->robot.runCommand(dropIntakePreload))
                            .build()
            ));

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(POS3_DUMP_X, DUMP_Y))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);
            // Repick
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, DUMP_Y+10))
                            .addTemporalMarker(0.1,()->robot.runCommand(fold))
                            .lineTo(new Vector2d(REPICK_X, REPICK_Y))
                            .build()
            ));
            robot.runCommand(repick);
            // goto pos 1 and dump there
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, DUMP_Y+15)) // goto pos 1
                            .addTemporalMarker(2.0,()->robot.runCommand(outPrep))
                            .splineTo(new Vector2d(POS1_DUMP_X-4,DUMP_Y-5),FACE_BACKDROP_HEADERING)
                            .build()
            ));
            alignBackdrop alignCmd2 = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
            robot.runCommand(alignCmd2);
            robot.runCommand(outDump);
        }

    }
}
