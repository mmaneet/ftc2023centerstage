package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "final teleop", group = "first")
public class finalTeleopGators extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Motor Declarations
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private DcMotorEx leftFront = null;
    private  DcMotorEx rightFront = null;

    private  DcMotorEx arm = null;

    private CRServo clawLeft = null;
    private CRServo clawRight = null;
    private DcMotorEx clawWrist = null;

    public void runOpMode() throws InterruptedException {
        try {
            //Telemetry Initialization ################################################################
            telemetry.setAutoClear(false);

            Telemetry.Item timeVal = telemetry.addData("Elapsed Time:", runtime);
            //------------------------------------------------------
            Telemetry.Item SPACER1 = telemetry.addData("---------------------", "-");
            //------------------------------------------------------
            Telemetry.Item leftEncode = telemetry.addData("Left Encoder:", 0);
            Telemetry.Item rightEncode = telemetry.addData("Right Encoder:", 0);
            Telemetry.Item leftEncodef = telemetry.addData("Left Encoder (Front):", 0);
            Telemetry.Item rightEncodef = telemetry.addData("Right Encoder (Front):", 0);
            //------------------------------------------------------
            Telemetry.Item SPACER2 = telemetry.addData("---------------------", "");
            //------------------------------------------------------
            Telemetry.Item armEncText = telemetry.addData("Arm Current Encoder Val:", "-");
            Telemetry.Item armPowerText = telemetry.addData("Arm Power:", "-");
            Telemetry.Item armTargetText = telemetry.addData("Current Arm Target:", "0");
            //------------------------------------------------------
            Telemetry.Item SPACER3 = telemetry.addData("---------------------", "");
            //------------------------------------------------------
            Telemetry.Item clawEncText = telemetry.addData("Claw Current Encoder Val:", "-");
            Telemetry.Item clawPowerText = telemetry.addData("Claw Power:", "-");
            Telemetry.Item clawTargetText = telemetry.addData("Claw Arm Target:", "0");
            //------------------------------------------------------
            Telemetry.Item SPACER4 = telemetry.addData("---------------------", "");
            //------------------------------------------------------
            Telemetry.Item G1LYText = telemetry.addData("G1 LY Val:", "-");
            Telemetry.Item G1LXText = telemetry.addData("G1 LX Val:", "-");
            Telemetry.Item G1RXText = telemetry.addData("G1 RX Val:", "-");
            //------------------------------------------------------
            Telemetry.Item SPACER5 = telemetry.addData("---------------------", "-");
            //------------------------------------------------------
            Telemetry.Item G2LYText = telemetry.addData("G2 LY Val:", "-");
            Telemetry.Item G2RYText = telemetry.addData("G2 RY Val:", "-");
            Telemetry.Item G2LTText = telemetry.addData("G2 LTrigger Val:", "-");
            Telemetry.Item G2RTText = telemetry.addData("G2 RTrigger Val:", "-");
            //##########################################################################################

            //Motor Initialization #####################################################################
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
            clawLeft = hardwareMap.get(CRServo.class, "clawLeft");
            clawRight = hardwareMap.get(CRServo.class, "clawRight");
            clawWrist = hardwareMap.get(DcMotorEx.class, "clawWrist");

            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setTargetPositionTolerance(10);

            clawWrist.setDirection(DcMotorSimple.Direction.FORWARD);
            clawWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            clawWrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            clawWrist.setTargetPosition(0);
            clawWrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawWrist.setTargetPositionTolerance(10);


            clawLeft.setDirection(CRServo.Direction.FORWARD);
            //##########################################################################################

            //PID Initialization########################################################################
            PID left_PID = new PID(leftBack.getVelocity(), 0.0, runtime);
            PID right_PID = new PID(rightBack.getVelocity(), 0.0, runtime);
            PID right_PID_f = new PID(rightFront.getVelocity(), 0.0, runtime);
            PID left_PID_f = new PID(leftFront.getVelocity(), 0.0, runtime);
            ArmPID armpid = new ArmPID(arm.getCurrentPosition(), 0);
            clawPID clawpid = new clawPID(clawWrist.getCurrentPosition(), 0);
            //##########################################################################################

            telemetry.addData("Status:", "Initalized");
            waitForStart();
            runtime.reset();
            while (opModeIsActive()) {
                // Velocity Vars
                double leftBackVelocity = leftBack.getVelocity();
                double rightBackVelocity = rightBack.getVelocity();
                double leftFrontVelocity = leftFront.getVelocity();
                double rightFrontVelocity = rightFront.getVelocity();
                int armPosition = arm.getCurrentPosition();
                int clawPosition = clawWrist.getCurrentPosition();

                //Inputs
                double G1LY = -gamepad1.left_stick_y;
                double G1LX = gamepad1.left_stick_x;
                double G1RX = gamepad1.right_stick_x;

                double G2LY = -gamepad2.left_stick_y;
                double G2RY = -gamepad2.right_stick_y;
                double lTrigger = gamepad2.left_trigger;
                double rTrigger = gamepad2.right_trigger;


                //Target Setting;
                double denominator = Math.max(Math.abs(G1LY) + Math.abs(G1LX) + Math.abs(G1RX), 1);
                double frontLeftPower = (G1LY + G1LX + G1RX) / denominator;
                double backLeftPower = (G1LY - G1LX + G1RX) / denominator;
                double frontRightPower = (G1LY - G1LX - G1RX) / denominator;
                double backRightPower = (G1LY + G1LX - G1RX) / denominator;

                double clawPower = rTrigger - lTrigger;
                int armPower;
                int clawWristPower;

                //Power Setting
                leftBack.setPower(backLeftPower);
                rightBack.setPower(backRightPower);
                leftFront.setPower(frontLeftPower);
                rightFront.setPower(frontRightPower);

                armPower = armpid.updatePID(armPosition, G2LY);
                arm.setPower(0.5);
                arm.setTargetPosition(armPower);

                clawWristPower = clawpid.updatePID(clawPosition, G2RY);
                clawWrist.setPower(0.5);
                clawWrist.setTargetPosition(clawWristPower);

                clawRight.setPower(clawPower);
                clawLeft.setPower(clawPower);

                //Telemetry Updates
                timeVal.setValue(getRuntime());

                leftEncode.setValue(leftBackVelocity);
                rightEncode.setValue(rightBackVelocity);
                leftEncodef.setValue(leftFrontVelocity);
                rightEncodef.setValue(rightFrontVelocity);

                armEncText.setValue(armPosition);
                armPowerText.setValue(armPower);
                armTargetText.setValue(arm.getTargetPosition());

                clawEncText.setValue(clawPosition);
                clawPowerText.setValue(clawPower);
                clawTargetText.setValue(clawWrist.getTargetPosition());

                G1LYText.setValue(G1LY);
                G1LXText.setValue(G1LX);
                G1RXText.setValue(G1RX);

                G2LYText.setValue(G2LY);
                G2RYText.setValue(G2RY);
                G2RTText.setValue(rTrigger);
                G2LTText.setValue(lTrigger);
                telemetry.update();

            }
        } catch (Exception ignored) {

        }
    }
}
