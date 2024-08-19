package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.driveAuto;

@TeleOp(name="Teleop Drive")
public class driveTeleop extends LinearOpMode {

    final private driveAuto armControl = new driveAuto();

    @Override
    public void runOpMode() {

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor armL = hardwareMap.dcMotor.get("armL");
        DcMotor armR = hardwareMap.dcMotor.get("armR");

        DcMotor flyL = hardwareMap.dcMotor.get("flyL");
        DcMotor flyR = hardwareMap.dcMotor.get("flyR");

        CRServo barrel = hardwareMap.crservo.get("barrel");

        ElapsedTime currentTime = new ElapsedTime();

        //TODO: test EVERYTHING before running to prevent damage

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        armL.setDirection(DcMotorSimple.Direction.FORWARD);
        armR.setDirection(DcMotorSimple.Direction.REVERSE);

        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //how many ms to wait between changing the angle of the arm
        double angleChangeDelay = 100;

        //how many degrees to change the arm angle
        double angleChangeRate = 5;

        double lastChange = 0.0;

        double currentArmAngle = -10;

        double upperArmLimit = 100;

        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive(leftFront,rightFront,leftBack,rightBack,1.0,gamepad1);

            if (gamepad1.right_bumper && (currentTime.milliseconds() > (lastChange + angleChangeDelay)) && !gamepad1.left_bumper && (currentArmAngle >= upperArmLimit)){
                currentArmAngle += angleChangeRate;
                lastChange = currentTime.milliseconds();
            }
            if (gamepad1.left_bumper && (currentTime.milliseconds() > (lastChange + angleChangeDelay)) && !gamepad1.right_bumper && (currentArmAngle >= currentArmAngle)){
                currentArmAngle -= angleChangeRate;
                lastChange = currentTime.milliseconds();
            }
            armControl.armAngle(currentArmAngle,armL,armR);

            armControl.spinFlywheel(gamepad1.right_trigger,flyL,flyR);


        }
    }

    public void mecanumDrive(DcMotor flMotor, DcMotor frMotor, DcMotor blMotor, DcMotor brMotor, double strafe_speed, Gamepad gamepad1){
        // get values from controller
        double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
        double stickX = gamepad1.left_stick_x*strafe_speed;
        double rStickX = gamepad1.right_stick_x;

        // get denominator
        double denominator = Math.max(Math.abs(stickX) + Math.abs(stickY) + Math.abs(rStickX), 1);
        // denominator ensures ratios are maintained, because the motors only go from 0-1

        // set values based on mecanum drive
        flMotor.setPower((stickY + stickX+ rStickX) / denominator);
        frMotor.setPower((stickY - stickX - rStickX) / denominator);
        blMotor.setPower((stickY - stickX + rStickX) / denominator);
        brMotor.setPower((stickY + stickX - rStickX) / denominator);
    }
}
