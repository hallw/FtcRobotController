package org.firstinspires.ftc.teamcode.subsystems;

import android.database.sqlite.SQLiteDoneException;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

@Config
//synchronizes lifts, provides commands to move to a position
public class DualMotorLift implements Subsystem {
    //Hardware: 2 lift motors
    private DcMotorEx slideMotorL;
    private DcMotorEx slideMotorR;
    private static final double TICKS_PER_REV = 751.8; //5203-2402-0027, 223 RPM
    private static final double PULLEY_DIAMETER_IN = (32.25 / 24.5); //3407-0002-0112 // = 1.269685 inches
    private final int HEIGHT_DIFF_TOLERANCE = inchToTicks(0.2); //(int) (0.3*TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    private Telemetry telemetry;
    private boolean targetReached = true;
    private final double FAST_POWER = 0.6;
    private final double SLOW_POWER = 0.3;
    private VoltageSensor batteryVoltageSensor;

    public enum Mode {
        BOTH_MOTORS_PID,
        RIGHT_FOLLOW_LEFT
    };
    public Mode mode;
    // Public just to allow tuning through Dashboard
    public static double  UP_VELOCITY = 500; // x inches per 1 second
    public static double[] LEVEL_HT = {13, 7, 17.0, 29.0}; // in inches, please fine-tune
    public static double[] LEVEL_HT_AUTO = {7, 4, 17.0, 29.0};
    //4 levels: 0 = minimum arm swing height; 1= ground, 2= low, 3= middle, 4= high,
    //0:5.0

    private PIDFController pidfController;
    public static double kP = 0.15;
    public static double kI = 0.0; //0.0000000001;
    public static double kD = 0.0;
    public static double kA = 0.0;
    public static double kV = 0.0;
    public static double kS = 0.002;
    public static double PID_RANGE = 0.9;
    public static double SLIDE_HOLD_POWER = 0.08;
    private double powerFromPIDF;


    //TODO: fine-tune LEVEL-HT values.
    public DualMotorLift (Robot robot, Telemetry telemetry, Mode mode){
        this.mode = mode;
        this.telemetry = telemetry;
        batteryVoltageSensor = robot.getVoltageSensor();
        slideMotorL = robot.getMotor("slideLt");
        slideMotorR = robot.getMotor("slideRt");
        slideMotorL.setTargetPositionTolerance(inchToTicks(0.3)); //HEIGHT_DIFF_TOLERANCE);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        //slideMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        if (mode== Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setTargetPosition(0);
            slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PIDCoefficients coefficients = new PIDCoefficients();
            coefficients.kP = kP;
            coefficients.kI = kI;
            coefficients.kD = kD;
            pidfController = new PIDFController(coefficients, kV, kA, kS);
            pidfController.setOutputBounds(-1.0*PID_RANGE, 1.0*PID_RANGE);
            pidfController.reset();
        }
        //Log.v("PIDLift: status: ", "init");
    }

    public void goToLevel(int level){
        //4 levels: 0 ground, 1 low, 2 middle, 3 high, 4 (minimum height for free chain bar movement)
        int targetPosition = inchToTicks(LEVEL_HT[level]);
        goToHt(targetPosition);
        //Log.v("ChainBar", "going to level" + level);

    }
    //for going to non-junction heights
    public void goToHt(int ticks) {
        targetReached=false;
        if(mode==Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotorR.setTargetPosition(ticks);
            slideMotorR.setVelocity(UP_VELOCITY);
            //fine tune velocity?
            //In case if i should set right motor right when i set left motor (prob not useful)
            slideMotorL.setVelocity(UP_VELOCITY);
        }
        else {
            pidfController.reset();
            pidfController.setTargetPosition(ticksToInches(ticks));
        }
        //Log.v("Lift: status", String.format("Lift moving to height %f", ticksToInches(slideMotorL.getTargetPosition())));
    }

    public void goToHtInches(double inches) {
        goToHt(inchToTicks(inches));
    }

    public void goToRelativeOffset(double inches) {
        targetReached=false;
        int currPosTicks = slideMotorR.getCurrentPosition();
        int targetPosTicks = currPosTicks + inchToTicks(inches);

        //Log.v("AUTOCMD DEBUG", "currPosTicks: " + currPosTicks);
        //Log.v("AUTOCMD DEBUG", "offset Inches: " + inches);
        //Log.v("AUTOCMD DEBUG", "targetPosTicks: " + targetPosTicks);
        this.goToHt(targetPosTicks);
    }

    public void applyStaticOffset(int direction, double power) {
        if(mode == Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotorR.setPower(power*direction);
        }
        else{
            slideMotorR.setPower(power*direction);
            slideMotorL.setPower(power*direction);
        }
        //Log.v("PIDLift: status: ", "applyStaticOffset");
    }

    public void adjustLift(int direction, boolean slow){
        targetReached=true;
        double power = SLOW_POWER;
        if(!slow){
            power = FAST_POWER;
        }
        if(mode == Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotorR.setPower(power*direction);
        }
        else{
            powerFromPIDF = power * direction;
            if (powerFromPIDF < PID_RANGE-SLIDE_HOLD_POWER) {
                powerFromPIDF += SLIDE_HOLD_POWER;
            } else if (powerFromPIDF < PID_RANGE) {
                powerFromPIDF = PID_RANGE;
            }
            slideMotorR.setPower(powerFromPIDF);
            slideMotorL.setPower(powerFromPIDF);
        }
        //Log.v("PIDLift: status: ", "adjust lift");
        //for using run-using-encoder mode
        //int current = slideMotorL.getCurrentPosition();
        // int targetPosition = current + inchToTicks(1)*direction;
        //slideMotorL.setTargetPosition(targetPosition);
    }

    public void stopMotor(){
        if(mode==Mode.RIGHT_FOLLOW_LEFT) {
            slideMotorL.setVelocity(0.0);
            slideMotorR.setVelocity(0.0);
        }
        slideMotorL.setPower(SLIDE_HOLD_POWER);
        slideMotorR.setPower(SLIDE_HOLD_POWER);
    }

    private void updateTargetReached() {
        //if it is already true, don't change it. only change when slide is set to a level
        double motorLVel;// = Math.abs(slideMotorL.getVelocity());
        double targetPos, currPos;

        motorLVel = Math.abs(slideMotorL.getVelocity());
        currPos = slideMotorL.getCurrentPosition();



        if (mode == Mode.RIGHT_FOLLOW_LEFT) {
            targetPos = slideMotorR.getTargetPosition();

            /*
            this.targetReached = (this.targetReached ||
                    (Math.abs(slideMotorL.getVelocity()) <= 20
                            && (Math.abs(slideMotorL.getTargetPosition() - slideMotorL.getCurrentPosition()) <= HEIGHT_DIFF_TOLERANCE)));
            */
        } else {
            targetPos = inchToTicks(pidfController.getTargetPosition());
            /*
            this.targetReached = (this.targetReached ||
                    (Math.abs(slideMotorL.getVelocity()) <= 20
                            && (Math.abs(inchToTicks(pidfController.getTargetPosition()) - slideMotorL.getCurrentPosition()) <= HEIGHT_DIFF_TOLERANCE)));
            */
        }
        this.targetReached = (this.targetReached || (motorLVel <= 20 && Math.abs(targetPos - currPos) <= HEIGHT_DIFF_TOLERANCE));
        //telemetry.addData("slideMotorL.getVelocity() ", Math.abs(slideMotorL.getVelocity()));
        //telemetry.addData("lastError ", ticksToInches((int)Math.abs(targetPos - currPos)));
        //telemetry.addData("targetReached ", this.targetReached);
        //telemetry.update();
    }

    public boolean isLevelReached(){
        return this.targetReached;
    }

    public int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    }
    public double ticksToInches(int ticks){
        return ((double) ticks) / (TICKS_PER_REV / (PULLEY_DIAMETER_IN * Math.PI));
    }

    public double getPosition(){
        return slideMotorR.getCurrentPosition() / (TICKS_PER_REV/(PULLEY_DIAMETER_IN * Math.PI));
    }

    public void resetEncoder(){
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getTargetPos(){
        return slideMotorR.getTargetPosition();
    }

    public boolean armCanSwing(){
        //Log.v("StateMach Arm: can swing?", ticksToInches(slideMotorR.getCurrentPosition())+"");
        return slideMotorR.getCurrentPosition()>=inchToTicks(LEVEL_HT[0]);
    }

    public boolean armCanSwingAuto(){
        //Log.v("StateMach Arm: can swing?", ticksToInches(slideMotorR.getCurrentPosition())+"");
        return slideMotorR.getCurrentPosition()>=inchToTicks(LEVEL_HT_AUTO[0]);
    }
    public double getPIDPower(){
        return powerFromPIDF;
    }

    @Override
    public void update(TelemetryPacket packet) {

        updateTargetReached();
        if(mode==Mode.RIGHT_FOLLOW_LEFT) {
            double velocity = slideMotorR.getVelocity();
            slideMotorL.setVelocity(velocity);
        }//if target is reached and not in manual mode, set velocity of right motor to 0
        else{
            if (!isLevelReached()) {
                double measuredPosition = (double) ticksToInches(slideMotorR.getCurrentPosition());
                powerFromPIDF = pidfController.update(measuredPosition);
                if (powerFromPIDF < PID_RANGE-SLIDE_HOLD_POWER) {
                    powerFromPIDF += SLIDE_HOLD_POWER;
                } else if (powerFromPIDF < PID_RANGE) {
                    powerFromPIDF = PID_RANGE;
                }
                //Log.v("PIDLift: Debug: ", String.format("Target pos: %4.2f, current pos: %4.2f, last error: %4.2f, velocity: %4.2f, set power to: %4.2f",
                //        pidfController.getTargetPosition(), measuredPosition, pidfController.getLastError(), slideMotorL.getVelocity(), powerFromPIDF));
                //telemetry.addData("Target pos", pidfController.getTargetPosition());
                //telemetry.addData("Measur pos", measuredPosition);
                //telemetry.addData("slidePower", powerFromPIDF);
                slideMotorL.setPower(powerFromPIDF);
                slideMotorR.setPower(powerFromPIDF);
            }
        }
        //telemetry.addLine("Slide motor set to " + ticksToInches(slideMotorL.getTargetPosition()));
        //telemetry.addLine("current slide velocity: " + slideMotorL.getVelocity());
        //telemetry.addLine("current slide position: " + ticksToInches(slideMotorR.getCurrentPosition()));
        packet.put("target pos (inches)", ticksToInches(slideMotorR.getTargetPosition()));
        if (mode == Mode.BOTH_MOTORS_PID) {
            packet.put("PID target pos", pidfController.getTargetPosition());
        }
        packet.put("left velocity", slideMotorL.getVelocity());
        packet.put("right velocity", slideMotorR.getVelocity());
        //Log.v("SLIDE L pos (inches)", ""+ticksToInches(slideMotorL.getCurrentPosition()));
        //Log.v("SLIDE R pos (inches)", ""+ticksToInches(slideMotorR.getCurrentPosition()));
        //Log.v("PIDLift: power", String.format("Left power: %f, Right power: %f",slideMotorL.getPower(), slideMotorR.getPower()));
        //Log.v("PIDLift: modes", String.format("left mode: %s, right mode: %s", slideMotorL.getMode().toString(), slideMotorR.getMode().toString()));
        //packet.put("motor power", slideMotorL.getPower());
        packet.put("R motor mode", slideMotorR.getMode());
        //FtcDashboard.getInstance().sendTelemetryPacket(packet);
        //telemetry.update();
    }
}
