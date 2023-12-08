//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.ServoControllerEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.*;
//
//import org.firstinspires.ftc.teamcode.PID;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.imgproc.Moments;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name="servoTester", group="First")
//
//public class servoTester extends LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//    CRServo clawLeft = null;
//    CRServo clawRight = null;
//
//    CRServo clawWrist = null;
//
//    DcMotorEx arm = null;
//
//
//
//    public void runOpMode() {
//        telemetry.setAutoClear(false);
//
//        Telemetry.Item timeVal = telemetry.addData("Elapsed Time", runtime);
//        Telemetry.Item armEnc = telemetry.addData("Arm Encoder Val:" , "-");
//        Telemetry.Item clrEnc = telemetry.addData("Claw Right Encoder Val:", "-");
//        Telemetry.Item cllEnc = telemetry.addData("Claw Left Encoder Val:", "-");
//        Telemetry.Item clwEnc = telemetry.addData("Claw Wrist Encoder Val:", "-");
//
//        Telemetry.Item aText = telemetry.addData("enc?:", "-");
//        Telemetry.Item bText = telemetry.addData("lastEnc?:", "-");
//        Telemetry.Item xText = telemetry.addData("err?:", "-");
//        Telemetry.Item yText = telemetry.addData("armPower?:", "-");
//
//        Telemetry.Item lyText = telemetry.addData("Left Stick Y Val:", "-");
//        Telemetry.Item lxText = telemetry.addData("Left Stick X Val:", "-");
//
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm.setDirection(DcMotorSimple.Direction.REVERSE);
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        clawLeft = hardwareMap.get(CRServo.class, "clawLeft");
//        clawRight = hardwareMap.get(CRServo.class, "clawRight");
//        clawWrist = hardwareMap.get(CRServo.class, "clawWrist");
//        clawLeft.setDirection(CRServo.Direction.REVERSE);
//
//
//        ArmPID2 armpid = new ArmPID(arm.getCurrentPosition(), 0);
//
//        waitForStart();
//        runtime.reset();
//        while (opModeIsActive()){
//            //g1: driving
//            //g2left: arm
//            //g2right: clawrist
//            //g2triggers: claw
//
//            boolean a = gamepad2.a;
//            boolean b = gamepad2.b;
//            boolean x = gamepad1.x;
//            boolean y = gamepad1.y;
//
//            double ly = -gamepad2.left_stick_y;
//            double ry = -gamepad2.right_stick_y;
//            double lTrigger = gamepad2.left_trigger;
//            double rTrigger = gamepad2.right_trigger;
//
//            double armPos = arm.getCurrentPosition();
//
////            if (a){
////                clawWrist.setPosition(0.1);
////            }
////            if (b){
////                clawWrist.setPosition(0.3);
////
////            }
////            if (x){
////                clawWrist.setPosition(0.6);
////            }
////            if (y){
////                clawWrist.setPosition(0.9);
////
////            }
//
//            double clawPower = rTrigger - lTrigger;
//            double armPower = armpid.updatePID(armPos, ly);
//            arm.setPower(armPower);
//            clawRight.setPower(clawPower);
//            clawLeft.setPower(clawPower);
//            clawWrist.setPower(ry);
//
//            if (a){
//                arm.setPower(0.3);
//            }
//            else if (b){
//                arm.setPower(-0.3);
//            }
//
//            timeVal.setValue(getRuntime());
//            armEnc.setValue(armPos);
//            clwEnc.setValue(clawWrist.getPower());
//            cllEnc.setValue(clawLeft.getPower());
//            clrEnc.setValue(clawRight.getPower());
//
//            aText.setValue(armpid.enc);
//            bText.setValue(armpid.lastEnc);
//            double err = armpid.enc - armpid.lastEnc;
////            if (Math.abs(err) > 0) {
////                xText.setValue(err);
////            }
////            else {
////                xText.setValue(false);
////            }
//            xText.setValue(armpid.mode);
//            yText.setValue(armPower);
//
//            lyText.setValue(ly);
//            telemetry.update();
//        }
//        }
//
//}
//
