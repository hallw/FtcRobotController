package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;

@Config
@TeleOp
public class TestArmIndividual extends LinearOpMode {
    public static int slideHt = 0;
    public static int slideHtStep = 1;
    public static double armRightIntake = 0.56; // - to move back
    public static double armLeftIntake = 0.59; //+ to move back
    public static double armStep = 0.01;

    public static double intakeL = 0.166; // start with intake pos
    public static double intakeR = 1 - intakeL;
    public static double intakeStep = 0.01;


    //armIntake_Right = 0.555; public static double armIntake_Left = 0.46;
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();
        robot.addGamepads(gamepad1, gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;
        RobotDistanceSensor distanceSensor = robot.ds;


        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();

        while (!isStopRequested()) {
            telemetry.update();
            robot.update();
            // Intake tests: start from intake position. gamepad1 x to retract, gamepad1 y to extend
            if (smartGamepad1.x_pressed()) {
                robot.intake.setIntakeState(100);
                intakeL += intakeStep;
                intakeR -= intakeStep;
            }
            if (smartGamepad1.y_pressed()) {
                robot.intake.setIntakeState(100);
                intakeL -= intakeStep;
                intakeR += intakeStep;
            }
            if (smartGamepad1.a_pressed()) { // cancel intake test
                robot.intake.setIntakeState(0);
            }
            robot.intake.intakeServoL.setPosition(intakeL);
            robot.intake.intakeServoR.setPosition(intakeR);
            // Outtake tests
            //Slide
            telemetry.addData("Slide Height ", slideHt);
            if(smartGamepad2.dpad_up_pressed()){
                robot.outtake.lift.goToHtInches(slideHt);
                slideHt += slideHtStep;
            }
            if(smartGamepad2.dpad_down_pressed()){
                robot.outtake.lift.goToHtInches(slideHt);
                slideHt -= slideHtStep;
            }
            //Arms
            //telemetry.addData("Arm Left ", armLeftIntake);

            if(smartGamepad2.x_pressed()){
                robot.outtake.armServo_Left.setPosition(armLeftIntake);
                armLeftIntake += armStep;
            }
            if(smartGamepad2.y_pressed()){
                robot.outtake.armServo_Left.setPosition(armLeftIntake);
                armLeftIntake -= armStep;
            }
            if(smartGamepad2.a_pressed()){
                robot.outtake.armServo_Right.setPosition(armRightIntake);
                armRightIntake -= armStep;
            }
            if(smartGamepad2.b_pressed()){
                robot.outtake.armServo_Right.setPosition(armRightIntake);
                armRightIntake += armStep;
            }
            telemetry.addData("Arm Left ", robot.outtake.armServo_Left.getPosition());
            telemetry.addData("Arm Right ", robot.outtake.armServo_Right.getPosition());
            telemetry.addData("intake L: ", robot.intake.intakeServoL.getPosition());
            telemetry.addData("intake R: ", robot.intake.intakeServoR.getPosition());
            double currentTime = clock.seconds();
            //telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
            }
        }
    }
