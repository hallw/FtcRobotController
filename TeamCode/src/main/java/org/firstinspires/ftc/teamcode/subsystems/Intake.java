package org.firstinspires.ftc.teamcode.subsystems;
import android.hardware.Sensor;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import java.net.CacheRequest;


public class Intake implements Subsystem {
    CrabRobot robot;
    //Hardware: 1 motor, 2 servo
    double syncFactor = 1.00;
    public double LPos, RPos;
    private DcMotorEx intakeMotor;
    private double motorPosition = 0;
    public Servo intakeServoL;
    public Servo intakeServoR;
    public DistanceSensor intakeTop; public DistanceSensor intakeBack;
    private double baseposl = 0.217;
    private double baseposr = 0.76715;
    private double baseposl_yield = 0.316;
    private double baseposr_yield = 1-baseposl_yield;
    private double outtakeposL = 0.206; //0.804;
    private double outtakeposR = 1-outtakeposL; //0.1503;
    //placeholder outtake position, may change depending on outtake
    private double intakeposL = 0.976; //0.031+0.02;
    private double intakeposR = 1-intakeposL; //0.9575-0.02;
    private double retakeposL = 0.896;
    private double retakeposR = 1-retakeposL;
    private double lowerlimitL = 0.814;
    private double lowerlimitR = 0.13;
    private double upperlimitL = 0.09;
    private double upperlimitR = 0.9;

    //State Machine
    public int intakeState = 0; //0: basePos, motor=0, 1: intakePos, motor=1, 2: outtakePos, motor=1
    private double motorDelayAfterOut = 2000; // in ms. after arm moves to outtake, delay to stop motor
    private double motorDelayForAuto = 1300; // in ms. after arm moves to outtake, delay to stop motor
    private double motorDelayForAutoOutput = 800;
    private double repickTime = 5000;
    private int intakeCount = 0;

    private double outtakeStartTime = 0;
    private double intakeStartTime = 0;
    private double intakeReverseStartTime = 0;
    private double motorSweepPwr = -1.0;
    private double autoOutputPwr = -0.4;
    public boolean isIntakeDone = false;

    public Intake(CrabRobot robot) {
        this.robot = robot;
        intakeMotor = robot.getMotor("intakeMotor");
        intakeServoL = robot.getServo("intakeServoL");
        intakeServoR = robot.getServo("intakeServoR");
        intakeTop = robot.hardwareMap.get(DistanceSensor.class, "intakeTop");
        intakeBack = robot.hardwareMap.get(DistanceSensor.class,"intakeBack");
        toBasePos();
    }
    public double getTopReading(){
        return intakeTop.getDistance(DistanceUnit.CM);
    }
    public double getBottomReading(){
        return intakeBack.getDistance(DistanceUnit.CM);
    }
    public void reset(){
        intakeServoL.setPosition(baseposl);
        intakeServoR.setPosition(baseposr);
    }
    public void toIntakePos(){
        intakeServoR.setPosition(intakeposR);
        intakeServoL.setPosition(intakeposL);
        //Log.v("intake", "to intake pos");
    }
    public void toOuttakePos(){
        intakeServoL.setPosition(outtakeposL);
        intakeServoR.setPosition(outtakeposR);
    }

    public void toBasePos(){
        intakeServoL.setPosition(baseposl);
        intakeServoR.setPosition(baseposr);
    }

    public void toBasePosYield(){
        intakeServoL.setPosition(baseposl_yield);
        intakeServoR.setPosition(baseposr_yield);
    }

    public void toRepickPos(){
        intakeServoR.setPosition(retakeposR);
        intakeServoL.setPosition(retakeposL);
    }

    public void moveArm(double d){
        double targetPosR = intakeServoR.getPosition()+(0.01*d*syncFactor);
        if(targetPosR>lowerlimitR&&targetPosR<upperlimitR) {
            intakeServoL.setPosition(intakeServoL.getPosition() + (0.01 * -d)); //2 degrees??
            intakeServoR.setPosition(intakeServoR.getPosition() + (0.01 * d * syncFactor));
        }
    }
    public void moveArmNoLimit(double d){
        intakeServoL.setPosition(intakeServoL.getPosition() + (0.04 * -d)); //2 degrees??
        intakeServoR.setPosition(intakeServoR.getPosition() + (0.04 * d * syncFactor));
    }
    public double getRightServoPos() {
        return intakeServoR.getPosition();
    }
    public double getLeftServoPos(){
        return intakeServoL.getPosition();
    }


    public void setIntakeState(int state) {
        this.intakeState = state;
    }
    public void setPower(double power) {
        this.motorPosition = -power;
        // set encode to new position
    }

    public double getPower() {
        return intakeMotor.getPower();
    }


    @Override
    public void update(TelemetryPacket packet) {

        if (intakeState == 0) {//Base, idle
            toBasePosYield();
            intakeMotor.setPower(0);
        } else if (intakeState == 1) {//Intake
            toIntakePos();
            intakeMotor.setPower(this.motorSweepPwr);
        } else if (intakeState == 2) {//Start outtake
            toOuttakePos();
            intakeMotor.setPower(this.motorSweepPwr);
            intakeState = 3;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 3) {//Done outtake
            long time = System.currentTimeMillis();
            if(time - outtakeStartTime >= this.motorDelayAfterOut){
                intakeState = 0;
            }
        } else if (intakeState == 11) {//Start output pre-load pixel
            toIntakePos();
            //intakeMotor.setPower(this.autoOutputPwr);
            intakeState = 12;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 12) { //wait for intake to intakePos
            long time = System.currentTimeMillis();
            if (time - outtakeStartTime >= this.motorDelayForAuto) {
                intakeState = 13;
                intakeMotor.setPower(-1*this.autoOutputPwr);
                outtakeStartTime = System.currentTimeMillis();
            }
        } else if (intakeState == 13) {//output
            if (System.currentTimeMillis() - outtakeStartTime >= this.motorDelayForAutoOutput) {
                intakeState = 0;
                intakeMotor.setPower(0);
                toBasePosYield();
            }
        } else if(intakeState == 21) { // Start reverse intake roller motor
            //intakeReverseStartTime = System.currentTimeMillis();
            intakeMotor.setPower(1);
            //intakeState = 8;
        } else if(intakeState == 22){
            intakeMotor.setPower(-1);
        }  else if (intakeState == 31) {//Start output pre-load pixel. AUTO ONLY, DO NOT CHANGE
            toIntakePos();
            //intakeMotor.setPower(this.autoOutputPwr);
            intakeState = 32;
            outtakeStartTime = System.currentTimeMillis();
        } else if (intakeState == 32) { //wait for intake to intakePos. AUTO ONLY, DO NOT CHANGE
            long time = System.currentTimeMillis();
            if (time - outtakeStartTime >= this.motorDelayForAuto) {
                intakeState = 33;
                intakeMotor.setPower(this.autoOutputPwr);
                outtakeStartTime = System.currentTimeMillis();
            }
        } else if (intakeState == 33) {//output, AUTO ONLY, DO NOT CHANGE
            if (System.currentTimeMillis() - outtakeStartTime >= this.motorDelayForAutoOutput) {
                intakeState = 0;
                intakeMotor.setPower(0);
                toBasePosYield();
            }

        } else if (intakeState == 40) {
            intakeServoL.setPosition(LPos);
        } else if (intakeState == 41) {
            intakeServoR.setPosition(RPos);

        } else if (intakeState == 51) {// repick Intake
            toRepickPos();
            intakeMotor.setPower(this.motorSweepPwr*0.95);
            intakeStartTime = System.currentTimeMillis();
            intakeState = 52;
        } else if (intakeState == 52) {//pick first
            if (intakeBack.getDistance(DistanceUnit.CM) < 5.0){
                intakeServoL.setPosition(retakeposL+0.015);
                intakeServoR.setPosition(retakeposR-0.015);
                intakeState = 53;
            }
        } else if (intakeState == 53) { //pick second
            if(intakeTop.getDistance(DistanceUnit.CM) < 7.5){
                intakeCount++;
            }
            else{
                intakeCount = 0;
            }
            if ((System.currentTimeMillis() - intakeStartTime >= this.repickTime)
                || intakeCount >= 3){
                isIntakeDone = true;
                setIntakeState(2);
            }
        }
        /*
        else if (intakeState == 100) { // test state

        }

         */






        //intakeMotor.setPower(motorPosition);
    }
}
