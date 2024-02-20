package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.alignBackdrop;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;

@TeleOp
@Config
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        DriveTrain drivetrain = new DriveTrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        DigitalChannel redLED;
        redLED = robot.hardwareMap.get(DigitalChannel.class, "redLed");
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setState(true);

        waitForStart();
        robot.addGamepads(gamepad1, gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;
        RobotDistanceSensor distanceSensor = robot.ds;
        TouchSensor sensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();
        int intakePosition = 0; // 0 = outtake; 1 = intake;
        int intakeCount = 0;
        double intakeStartTime = 0;

        boolean inAlignCmd = false;
        boolean stopAlign = false;


        while (!isStopRequested()) {
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            robot.update();

            boolean slowMode = gamepad1.left_bumper;
            double joystickRadius = Math.min(1,Math.sqrt(Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.left_stick_x,2)));
            //double factor = robot.mecanumDrive.mapJsRadiusVal(joystickRadius,slowMode);
            double jsX = robot.mecanumDrive.mapJsComponents(-gamepad1.left_stick_x, joystickRadius, slowMode);
            double jsY = robot.mecanumDrive.mapJsComponents(gamepad1.left_stick_y, joystickRadius, slowMode);

            if(!inAlignCmd || stopAlign) {
                robot.mecanumDrive.setDrivePower(new Pose2d(-jsY, -jsX, -(0.8) * gamepad1.right_stick_x));
                robot.mecanumDrive.setPowerFactor(0.7); //remove with actual robot.
            }

            // LED
            redLED.setState(!inAlignCmd);

            // do not move
            if(smartGamepad1.right_bumper == false && robot.intake.intakeState == 21){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 0;
            }
            if(smartGamepad1.right_bumper){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 21;
            }
            if(smartGamepad1.right_trigger < 0.5 && robot.intake.intakeState == 22){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 0;
            }
            if(smartGamepad1.right_trigger >= 0.5){
                robot.intake.intakeState = 22;
            }

            // INTAKE
            /*if(smartGamepad1.a_pressed()){
                if(intakePosition == 0 && robot.intake.intakeState == 0) {
                    robot.intake.setIntakeState(1);
                    intakePosition=1;
                }
                else{
                    robot.intake.setIntakeState(2);
                    intakePosition = 0;
                    intakeCount = 0;
                }
            }
            else if (intakePosition == 1 && robot.intake.intakeState == 1) {
                if(robot.intake.intakeTop.getDistance(DistanceUnit.CM) < 7.5){
                    intakeCount++;
                }
                else{
                    intakeCount = 0;
                }
                if (intakeCount >= 3) {
                    //lifts up the intake only when it is intaking and if intakeCount >= 3 or a is pressed
                    robot.intake.setIntakeState(2);
                    intakePosition = 0;
                    intakeCount = 0;
                }
            }
             */
            if(intakePosition == 0 && robot.intake.intakeState == 0){//intake idle
                if(smartGamepad1.a_pressed()){
                    robot.intake.setIntakeState(1);
                    intakePosition = 1;
                }
            }
            else if(intakePosition == 1 && robot.intake.intakeState == 1){//intaking the pixels
                if(robot.intake.intakeTop.getDistance(DistanceUnit.CM) < 7.5){
                    intakeCount++;
                }
                else{
                    intakeCount = 0;
                }
                if (intakeCount >= 3 || smartGamepad1.a_pressed()) {
                    //lifts up the intake only when it is intaking and if intakeCount >= 3 or a is pressed
                    robot.intake.setIntakeState(2);
                    intakePosition = 0;
                    intakeCount = 0;
                }
            }

            if(smartGamepad1.b_pressed()){
                robot.intake.toBasePos();
            }
            if(smartGamepad1.x_pressed()){ // test auto output command
                robot.intake.intakeState = 11;
            }
            if(smartGamepad1.dpad_up_pressed()){
                if (!inAlignCmd) {
                    alignBackdrop alignCmd = new alignBackdrop(robot, drivetrain, 0.2, 1,9, telemetry);
                    inAlignCmd = true;
                    Log.v("Align", "Align called");

                    robot.runCommand(alignCmd);
                } else if(inAlignCmd) {
                    Log.v("Align", "Exit AlignCmd");
                    telemetry.addLine("Exit Align");
                    inAlignCmd = false;
                }
            }
            telemetry.addData("inAlignCmd", inAlignCmd);
            if(smartGamepad1.dpad_down_pressed()){
                stopAlign = !stopAlign;
            }


            // Outtake automated
            if(smartGamepad2.a_pressed()){
                robot.intake.setPower(0);
                robot.intake.toBasePos();
                robot.outtake.prepOuttake();
            }

            if (smartGamepad2.right_bumper) {
                robot.outtake.dropPixelPos();
            }

            if (smartGamepad2.x_pressed()) {
                robot.outtake.downToIntakePos();
            }

            // Drone launcher
            if(smartGamepad2.left_bumper && smartGamepad1.y){
                robot.droneLauncher.release();
            }

                //UG OUTTAKE, single steps
            if(smartGamepad2.left_trigger>0) { robot.outtake.moveDumper(-0.5);}
            if(smartGamepad2.right_trigger>0) {robot.outtake.moveDumper( 0.5);}

            if (smartGamepad2.dpad_right) {
                robot.outtake.moveArm(0.5);
            }
            if (smartGamepad2.dpad_left) {
                robot.outtake.moveArm(-0.5);
            }

            if(smartGamepad2.b_pressed()){
                robot.outtake.toDumpPos();
            }

            if (smartGamepad2.y_pressed()) {
                robot.outtake.armToTravelPos();
                robot.outtake.dumperToIntakePos();
            }

            if(smartGamepad2.right_stick_button){
                    robot.outtake.prepHang();
            }
            if(smartGamepad2.leftJoystickButton() || sensor.isPressed()){
                robot.outtake.lift.resetEncoder();
            }

            if (smartGamepad2.dpad_up) {
                robot.outtake.lift.adjustLift(1, false);
                //telemetry.addLine("dpad up pressed");
                //Log.v("PIDLift: gamepad", "dpad up");
            }
            else if (smartGamepad2.dpad_down) { 
                robot.outtake.lift.adjustLift(-1, false);
                //telemetry.addLine("dpad down pressed");
                //Log.v("PIDLift: gamepad", "dpad down");
            } else if (robot.outtake.lift.isLevelReached()){
                robot.outtake.lift.stopMotor();
            }

            //telemetry.addData("intake pos", intakePosition);
            //telemetry.addData("intake motor power", robot.intake.getPower());

            //telemetry.addData("right servo position: ", robot.outtake.get_RightServoPos());
            //telemetry.addData("left servo position: ", robot.outtake.get_LeftServoPos());
            //telemetry.addData("dumper servo position: ", robot.outtake.getDumperPos());
            //telemetry.addData("slide pos", robot.outtake.getLiftPos());
            //telemetry.addData("slide power", robot.outtake.getLiftPower());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());
            telemetry.addData("DistR: ",distanceSensor.distanceRight());
            telemetry.addData("DistL: ",distanceSensor.distanceLeft());
            telemetry.addData("intakeTop: ", robot.intake.intakeTop.getDistance(DistanceUnit.CM));
            telemetry.addData("intakeBack: ", robot.intake.intakeBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Slide Encoder", robot.outtake.lift.getLeftEncoder());
            telemetry.addData("Right Slide Encoder", robot.outtake.lift.getRightEncoder());
            packet.put("Left Slide Encoder", robot.outtake.lift.getLeftEncoder());
            packet.put("Right Slide Encoder", robot.outtake.lift.getRightEncoder());
            double currentTime = clock.seconds();
            //telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
            }
        }
    }
