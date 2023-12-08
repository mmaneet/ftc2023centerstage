package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "blueBackdropSideAuto", group = "Blue")
public class blueBackdropSideAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Motor Declarations
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private DcMotorEx leftFront = null;
    private  DcMotorEx rightFront = null;
    private CRServo clawLeft = null;
    private DcMotorEx arm = null;
    private DcMotorEx clawWrist = null;



    public void turn(double pow, int time){
        runtime.reset();
        while (runtime.seconds() < time) {
            leftBack.setPower(pow);
            rightBack.setPower(-pow);
            leftFront.setPower(pow);
            rightFront.setPower(-pow);
        }
        //neg = left, pos = right
    }

    public void moveStraight(double pow, int time){
        runtime.reset();
        while (runtime.seconds() < time) {
            leftBack.setPower(pow);
            rightBack.setPower(pow);
            leftFront.setPower(pow);
            rightFront.setPower(pow);
        }
    }
    public void dropPixel(double time){
        runtime.reset();
        while (runtime.seconds() < time) {
            clawLeft.setPower(0.5);
        }
    }
    public void stopBot(){
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void moveArm(int pos){
        arm.setTargetPosition(pos);
        while (arm.isBusy()){

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBackWheel");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBackWheel");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontWheel");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontWheel");

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        arm.setTargetPositionTolerance(10);

        clawWrist = hardwareMap.get(DcMotorEx.class, "clawWrist");
        clawWrist.setDirection(DcMotorEx.Direction.FORWARD);
        clawWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawWrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawWrist.setTargetPosition(0);
        clawWrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawWrist.setTargetPositionTolerance(10);

        clawLeft = hardwareMap.get(CRServo.class, "clawLeft");
        clawLeft.setDirection(CRServo.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        turn(-0.53, 2);
        moveStraight(0.5, 5);
        stopBot();
        moveArm(1700);
        dropPixel(3);
        moveArm(0);


    }
}
