package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="motor test")
public class motorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double motorPower = 0.2;

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor armL = hardwareMap.dcMotor.get("armL");
        DcMotor armR = hardwareMap.dcMotor.get("armR");

        DcMotor flyL = hardwareMap.dcMotor.get("flyL");
        DcMotor flyR = hardwareMap.dcMotor.get("flyR");

        CRServo barrel = hardwareMap.crservo.get("barrel");



        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("A Button",leftFront.getConnectionInfo());
            if (gamepad1.a) {
                leftFront.setPower(motorPower);
            }

            telemetry.addData("B Button",rightFront.getConnectionInfo());
            if (gamepad1.b) {
                rightFront.setPower(motorPower);
            }

            telemetry.addData("X Button",leftBack.getConnectionInfo());
            if (gamepad1.x) {
                leftBack.setPower(motorPower);
            }

            telemetry.addData("Y Button",rightBack.getConnectionInfo());
            if (gamepad1.y) {
                rightBack.setPower(motorPower);
            }

            telemetry.addData("D-Pad Left",armL.getConnectionInfo());
            if (gamepad1.dpad_left) {
                armL.setPower(motorPower);
            }

            telemetry.addData("D-Pad Right",armR.getConnectionInfo());
            if (gamepad1.dpad_right) {
                armR.setPower(motorPower);
            }

            telemetry.addData("D-Pad Up",flyL.getConnectionInfo());
            if (gamepad1.dpad_up) {
                flyL.setPower(motorPower);
            }

            telemetry.addData("D-Pad Down",flyR.getConnectionInfo());
            if (gamepad1.dpad_down) {
                flyR.setPower(motorPower);
            }

            telemetry.addData("Right Bumper",barrel.getConnectionInfo());
            if (gamepad1.right_bumper) {
                barrel.setPower(motorPower);
            }
        }
    }
}
