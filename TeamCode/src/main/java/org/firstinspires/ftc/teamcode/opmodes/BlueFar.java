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
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.util.Utilities;

import java.util.Vector;

@Config
@Autonomous
public class BlueFar extends LinearOpMode {
    public static boolean parkCenter = true; // Park center of field
    public static boolean IS_RED = false;     // IS_RED side?
    public static boolean ALIGN_RIGHT = true; // Align 1 inch from tile right side

    public static double drivePwr = 0.2;
    public static double hCoeff = 1;
    public static double park_y = 75;


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
        //Servo init code here
        alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, drivePwr, hCoeff,9, telemetry);
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
        //Log.v("AUTODEBUG", "1: elementPos = " + elementPos);
        telemetry.addData("Element pos", elementPos);
        telemetry.addData("Is parking center?: ", parkCenter);
        telemetry.update();


        if (elementPos == 1) {//left
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(28, -5), Math.toRadians(-90))
                            .lineTo(new Vector2d(28,-3))
                            .addTemporalMarker(2.0,()->robot.runCommand(dropIntakePreload))
                            .build()
            ));
            //Log.v("AUTODEBUG", "2: dump purple");
            //Log.v("AUTODEBUG", "3: go to backdrop");

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .turn(Math.toRadians(45), 1,0.5)
                            .splineTo(new Vector2d(5.5, 2), Math.toRadians(90))
                            .splineTo(new Vector2d(5.5, 63), Math.toRadians(90))// go to backdrop
                            //.lineTo(new Vector2d(2, -74))
                            .build()
            ));
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(25, park_y), Math.toRadians(90))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);

            //Log.v("AUTODEBUG", "4: dump yellow");
            //dump yellow pixel
            //Log.v("AUTODEBUG", "5: park");
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(50, park_y+5))
                                .addTemporalMarker(0.1, ()->robot.runCommand(fold))
                                .forward(11)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(5, park_y+5))
                                .addTemporalMarker(0.1, ()->robot.runCommand(fold))
                                .forward(11)
                                .build()
                ));
            }

        } else if (elementPos == 2) { //middle
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(21, -3), Math.toRadians(-180))
                            .build()
            ));
            robot.runCommand(dropIntakePreload);
            // go to back drop
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(5.5, 2), Math.toRadians(90))
                            .splineTo(new Vector2d(5.5, 60), Math.toRadians(90))
                            .build()
            ));
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(29, park_y+2), Math.toRadians(90))
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
                                .lineTo(new Vector2d(50, park_y+5))
                                .addTemporalMarker(0.1, ()->robot.runCommand(fold))
                                .forward(11)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(5, park_y+5))
                                .addTemporalMarker(0.1, ()->robot.runCommand(fold))
                                .forward(11)
                                .build()
                ));
            }

        } else {// right

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .splineTo(new Vector2d(14, -9), Math.toRadians(180))
                            .addTemporalMarker(2.0,()->robot.runCommand(dropIntakePreload))
                            .build()
            ));
            //Log.v("AUTODEBUG", "2: dump purple");
            //Log.v("AUTODEBUG", "3: go to backdrop");

            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(5.5, 2), Math.toRadians(90))
                            .splineTo(new Vector2d(5.5, 60), Math.toRadians(90))// go to backdrop
                            //.lineTo(new Vector2d(2, -74))
                            .build()
            ));
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(34.5,  park_y), Math.toRadians(90))
                            .addTemporalMarker(0.2, () -> robot.runCommands(outPrep))
                            .build()
            ));
            //align
            robot.runCommand(alignCmd);
            //dump yellow pixel
            robot.runCommand(dumpNBack);
            // Park
            if(parkCenter){
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(50, park_y+5))
                                .addTemporalMarker(0.1, ()->robot.runCommand(fold))
                                .forward(11)
                                .build()
                ));
            }
            else{
                robot.runCommand(drivetrain.followTrajectorySequence(
                        drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                                .lineTo(new Vector2d(5, park_y+5))
                                .addTemporalMarker(0.1, ()->robot.runCommand(fold))
                                .forward(11)
                                .build()
                ));
            }


        }

    }
}
