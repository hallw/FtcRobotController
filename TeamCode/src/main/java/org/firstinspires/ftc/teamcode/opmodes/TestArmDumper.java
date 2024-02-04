package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;

@Config
@TeleOp
public class TestArmDumper extends LinearOpMode {
    public static double dumperPos = 0.35;  // - to turn backwards
    public static double dumperPosStep = 0.01;
    public static int slideHt = 0;
    public static int slideHtStep = 1;
    public static double armLeftIntake = 0.54; //+ to move back
    public static double armRightIntake = 0.44; // - to move back
    public static double intakeServoLeftPos = 0.36 + 0.37;
    public static double intakeServoRightPos = 0.615 - 0.37;
    public static double armStep = 0.01;

    public static double intakeStep = 0.1;

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
        int intakePosition = 0; // 0 = outtake; 1 = intake;
        double intakeStartTime = 0;

        while (!isStopRequested()) {
            telemetry.update();
            robot.update();

            //boolean slowMode = gamepad1.left_bumper;
            //double joystickRadius = Math.min(1,Math.sqrt(Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.left_stick_x,2)));
            //double factor = robot.mecanumDrive.mapJsRadiusVal(joystickRadius,slowMode);
            //double jsX = robot.mecanumDrive.mapJsComponents(-gamepad1.left_stick_x, joystickRadius, slowMode);
            //double jsY = robot.mecanumDrive.mapJsComponents(gamepad1.left_stick_y, joystickRadius, slowMode);
            //robot.mecanumDrive.setDrivePower(new Pose2d(-jsY, jsX, -(0.8)*gamepad1.right_stick_x));
            //robot.mecanumDrive.setPowerFactor(0.7); //remove with actual robot.

            // do not move
            // INTAKE
            if(smartGamepad1.a_pressed()){
                if(intakePosition == 0 && robot.intake.intakeState == 0) {
                    robot.intake.setIntakeState(1);
                    intakePosition=1;
                } else if (intakePosition == 1 && robot.intake.intakeState == 1){
                    robot.intake.setIntakeState(2);
                    intakePosition=0;
                }
            }
            if(smartGamepad1.b_pressed()){
                robot.intake.toBasePos();
            }
            if(smartGamepad1.x_pressed()){ // test auto output command
                robot.intake.intakeState = 11;
            }
            telemetry.addData("Intake Left Servo Position:",intakeServoLeftPos);
            telemetry.addData("Intake Right Servo Position:",intakeServoRightPos);
            if(smartGamepad1.dpad_left_pressed()){
                robot.intake.LPos = intakeServoLeftPos;
                intakeServoLeftPos -= intakeStep;
            }
            if(smartGamepad1.dpad_right_pressed()){
                robot.intake.LPos = intakeServoLeftPos;
                intakeServoLeftPos += intakeStep;
                //robot.intake.intakeServoR.setPosition(intakeServoRightPos);
                //intakeServoRightPos -= intakeStep;
            }

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
            telemetry.addData("Arm Left ", armLeftIntake);
            telemetry.addData("Arm Right ", armRightIntake);
            if(smartGamepad2.x_pressed()){
                robot.outtake.armServo_Left.setPosition(armLeftIntake);
                robot.outtake.armServo_Right.setPosition(armRightIntake);
                armLeftIntake += armStep;
                armRightIntake -= armStep;
            }
            if(smartGamepad2.y_pressed()){
                robot.outtake.armServo_Left.setPosition(armLeftIntake);
                robot.outtake.armServo_Right.setPosition(armRightIntake);
                armLeftIntake -= armStep;
                armRightIntake += armStep;
            }
            // Dumper
            telemetry.addData("DumperPos ",dumperPos);
            if(smartGamepad2.a_pressed()){
                robot.outtake.setDumpServoPos(dumperPos);
                dumperPos += dumperPosStep;
            }

            if(smartGamepad2.b_pressed()){
                robot.outtake.setDumpServoPos(dumperPos);
                dumperPos -= dumperPosStep;
            }


            telemetry.addData("DistR: ",distanceSensor.distanceRight());
            telemetry.addData("DistL: ",distanceSensor.distanceLeft());
            double currentTime = clock.seconds();
            //telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
            }
        }
    }
