package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.commands.backAfterDump;
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.commands.autoOutPrep;
import org.firstinspires.ftc.teamcode.commands.autoOutDump;
import org.firstinspires.ftc.teamcode.commands.foldOuttake;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;

@Config
@Autonomous
public class RedNear extends LinearOpMode {
    public static boolean parkCenter = true;
    public static boolean IS_RED = true;
    public static boolean ALIGN_RIGHT = true;
    public static double POS1_SPL1_X = 24;
    public static double POS1_SPL1_Y = -4;
    public static double POS1_DUMP_X = 31;
    public static double DUMP_Y = -27;

    public static double POS2_SPL1_X = 37;
    public static double POS2_SPL1_Y = -20;
    public static double POS3_SPL1_X = 26;
    public static double POS3_SPL1_Y = -24;
    public static double FACE_BACKDROP_HEADERING = Math.toRadians(-90);
    public static double PARK_CENTER_X = 50;
    public static double PARK_CORNER_X = 0;
    public static double PARK_STRAFE_MIDDLE_TO_CENTER = 28;
    public static double TAG_DIST = 6;
    public static double PARK_FORWARD = 10.0;
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

            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-5))
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-13))
                                .addTemporalMarker(0.1,()->robot.runCommand(fold))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-5))
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-13))
                                .addTemporalMarker(0.1,()->robot.runCommand(fold))
                                .build()
                ));
            }

        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(POS2_SPL1_X, POS2_SPL1_Y), FACE_BACKDROP_HEADERING)
                            .addTemporalMarker(1.0,()->robot.runCommands(dropIntakePreload))
                            .build()
            ));
            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(24, DUMP_Y))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-5))
                                .addTemporalMarker(0.1,()->robot.runCommand(fold))
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-5))
                                .addTemporalMarker(0.1,()->robot.runCommand(fold))
                                .forward(PARK_FORWARD)
                                .build()
                ));
            }

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
                            .lineTo(new Vector2d(18, DUMP_Y))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CENTER_X, DUMP_Y-5))
                                .forward(PARK_FORWARD)
                                .addTemporalMarker(0.1,()->robot.runCommand(fold))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_CORNER_X, DUMP_Y-12))
                                .forward(PARK_FORWARD)
                                .addTemporalMarker(0.1,()->robot.runCommand(fold))
                                .build()
                ));
            }
        }

    }
}
