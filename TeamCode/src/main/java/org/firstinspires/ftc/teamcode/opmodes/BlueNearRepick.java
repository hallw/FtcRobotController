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
import org.firstinspires.ftc.teamcode.commands.foldOuttake;
import org.firstinspires.ftc.teamcode.commands.repick;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class BlueNearRepick extends LinearOpMode {
    public static boolean parkCenter = false; // Park center of field
    public static boolean IS_RED = false;     // IS_RED side?
    public static boolean ALIGN_RIGHT = false; // Align 1 inch from tile right side
    public static double FACE_BACKDROP_HEADERING = Math.toRadians(90);
    public static double POS1_SPL_X = 27; public static double POS1_SPL_Y = 23.5;
    public static double POS1_DUMP_X = 18;
    public static double POS2_SPL_X = 37; public static double POS2_SPL_Y = 20;
    public static double POS2_DUMP_X = 25;
    public static double POS3_SPL_X = 28; public static double POS3_SPL_Y = 6;
    public static double POS3_DUMP_X = 31;
    public static double PARK_X_CORNER = 0;
    public static double PARK_X_CENTER = 53;
    public static double PARK_Y = 34.5;
    public static double REPICK_X = 48;
    public static double REPICK_Y = -58.5;

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
        alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
        //Servo init code here
        robot.intake.toBasePos();
        robot.outtake.toIntakePos();
        dropIntakePreload dropIntakePreload = new dropIntakePreload(robot);
        autoOutPrep outPrep = new autoOutPrep(robot,false);
        autoOutPrep outPrepr = new autoOutPrep(robot,true);
        backAfterDump back = new backAfterDump(robot,drivetrain);
        foldOuttake fold = new foldOuttake(robot,drivetrain);
        repick repick = new repick(robot,drivetrain);
        autoOutDump outDump = new autoOutDump(robot, drivetrain);


        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        // Start
        telemetry.addData("Is parking center?: ", parkCenter);
        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;
        //Log.v("AUTODEBUG", "0: start");
        elementPos = rvis.getTeamPropOrientation(IS_RED, ALIGN_RIGHT);
        //Log.v("AUTODEBUG", "1: elementPos = %0d");
        //telemetry.addData("Element pos", elementPos);
        telemetry.addData("Is parking center?: ", parkCenter);
        telemetry.addData("Detected Pos:", elementPos);
        telemetry.update();


        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS1_SPL_X, POS1_SPL_Y), FACE_BACKDROP_HEADERING)//22
                            .addTemporalMarker(1.0,()->robot.runCommands(dropIntakePreload))
                            .build()
            ));

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(POS1_DUMP_X, PARK_Y))
                            .addTemporalMarker(0.0, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            robot.runCommand(back);
            //dump yellow pixel
            //Log.v("AUTODEBUG", "10: dump done");

            // Repick
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, PARK_Y-5))
                            .addTemporalMarker(0.1,()->robot.runCommand(fold))
                            .lineTo(new Vector2d(REPICK_X, REPICK_Y))
                            .build()
            ));
            robot.runCommand(repick);
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, PARK_Y-10))
                            .addTemporalMarker(2.0,()->robot.runCommand(outPrepr))
                            .splineTo(new Vector2d(POS3_DUMP_X,PARK_Y-5),FACE_BACKDROP_HEADERING) // goto pos 3
                            .build()
            ));
            alignBackdrop alignCmd2 = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
            robot.runCommand(alignCmd2);
            robot.runCommand(outDump);
        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS2_SPL_X, POS2_SPL_Y), FACE_BACKDROP_HEADERING)
                            .addTemporalMarker(2.0,()->robot.runCommands(dropIntakePreload)) // dump purple pixel
                            .build()
            ));

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(POS2_DUMP_X, PARK_Y)) //24 previous
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));

            robot.runCommand(alignCmd);
            robot.runCommand(back); //dump yellow and back

            // Repick
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, PARK_Y-5))
                            .addTemporalMarker(0.1,()->robot.runCommand(fold))
                            .lineTo(new Vector2d(REPICK_X, REPICK_Y))
                            .build()
            ));
            robot.runCommand(repick);
            // goto pos 3 and dump there
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, PARK_Y-7))
                            .addTemporalMarker(2.0,()->robot.runCommand(outPrepr))
                            .splineTo(new Vector2d(POS3_DUMP_X,PARK_Y-3),FACE_BACKDROP_HEADERING) // goto pos 3
                            .build()
            ));
            alignBackdrop alignCmd2 = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
            robot.runCommand(alignCmd2);
            robot.runCommand(outDump);

        } else {// pos 3: right
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS3_SPL_X, POS3_SPL_Y), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(POS3_SPL_X, POS3_SPL_Y-3.5))
                            .addTemporalMarker(2.8,()->robot.runCommands(dropIntakePreload)) // dump purple pixel
                            .build()
            ));

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(POS3_DUMP_X, PARK_Y))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            robot.runCommand(back); //dump yellow and back

            // Repick
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, PARK_Y-5))
                            .addTemporalMarker(0.1,()->robot.runCommand(fold))
                            .lineTo(new Vector2d(REPICK_X, REPICK_Y))
                            .build()
            ));
            robot.runCommand(repick);
            // goto pos 1 and dump there
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(REPICK_X, PARK_Y-15 ))
                            .addTemporalMarker(2.0,()->robot.runCommand(outPrepr))
                            .splineTo(new Vector2d(POS1_DUMP_X+2,PARK_Y-5),FACE_BACKDROP_HEADERING) // goto pos 1
                            .build()
            ));
            alignBackdrop alignCmd2 = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,8, telemetry);
            robot.runCommand(alignCmd2);
            robot.runCommand(outDump);
        }

    }
}

