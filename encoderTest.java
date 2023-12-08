package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="encoder test", group="First")
public class encoderTest extends LinearOpMode {
    DcMotorEx t = null;

    public void runOpMode(){

        Telemetry.Item encoderVal =  telemetry.addData("Encoder Val:", 0);
        t = hardwareMap.get(DcMotorEx.class, "t");
        t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        t.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()){
            double p = gamepad1.left_stick_y;
            double vel = t.getVelocity();
            t.setPower(p);
            encoderVal.setValue(vel);
            telemetry.update();
        }
    }
}
