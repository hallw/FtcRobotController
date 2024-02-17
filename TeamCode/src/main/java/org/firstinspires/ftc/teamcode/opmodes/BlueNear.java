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
import org.firstinspires.ftc.teamcode.commands.dropIntakePreload;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Config
@Autonomous
public class BlueNear extends LinearOpMode {
    public static boolean parkCenter = false; // Park center of field
    public static boolean IS_RED = false;     // IS_RED side?
    public static boolean ALIGN_RIGHT = false; // Align 1 inch from tile right side
    public static double POS1_SPL1_X = 23;
    public static double POS1_SPL1_Y = 24;
    public static double POS1_DUMP_X = 24;
    public static double POS1_DUMP_Y = 33;

    public static double POS2_SPL1_X = 37;
    public static double POS2_SPL1_Y = 15;
    public static double POS3_SPL1_X = 26;
    public static double POS3_SPL1_Y = 26;
    public static double FACE_BACKDROP_HEADERING = Math.toRadians(90);
    public static double PARK_STRAFE_MIDDLE_TO_CENTER = 28;
    public static double TAG_DIST = 6;
    public static double PARK_X_CORNER = 0;
    public static double PARK_X_CENTER = 53;
    public static double PARK_Y = 40;
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
        autoOutPrep outCmd = new autoOutPrep(robot);
        autoOutPrep outPrep = new autoOutPrep(robot);
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


        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(29, 24), FACE_BACKDROP_HEADERING)//22
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(20, PARK_Y))
                            .addTemporalMarker(0.5, () -> robot.runCommands(outPrep))
                            .build()
            ));
            //robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            //Log.v("AUTODEBUG", "10: dump done");

            // Park
            if(!parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_X_CORNER, PARK_Y+5))
                                //.lineTo(new Vector2d(2, 4))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_X_CENTER, PARK_Y+5))
                                .build()
                ));
            }

        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(37, 18), FACE_BACKDROP_HEADERING)
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(25, PARK_Y)) //24 previous
                            .addTemporalMarker(0.5, () -> robot.runCommands(outPrep))
                            .build()
            ));
            //robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            // Park
            if(!parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_X_CORNER, PARK_Y+5))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_X_CENTER, PARK_Y+5))
                                .build()
                ));
            }

        } else {// right
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(28, 6), FACE_BACKDROP_HEADERING)
                            .lineTo(new Vector2d(26, 2))
                            .build()
            ));
            //dump purple pixel
            robot.runCommand(dropIntakePreload);

            // go to back drop
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            .lineTo(new Vector2d(32, PARK_Y))
                            .addTemporalMarker(0.5, () -> robot.runCommands(outPrep))
                            .build()
            ));
            //robot.runCommand(outPrep);
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(outDump);
            // Park
            if(!parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(PARK_X_CORNER, PARK_Y+5))
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(50, PARK_Y+5))
                                .build()
                ));
            }
        }

    }
}

