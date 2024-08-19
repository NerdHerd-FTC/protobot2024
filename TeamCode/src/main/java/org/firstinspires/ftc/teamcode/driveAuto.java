package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Drive Auto")
public class driveAuto extends LinearOpMode {

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

        telemetry.addData("armL position",armL.getCurrentPosition());
        telemetry.addData("armR position",armR.getCurrentPosition());
        telemetry.update();

        waitForStart();
        driveToLocation(2,3,leftFront,rightFront,leftBack,rightBack);
        armAngle(30,armL,armR);
    }

    public void driveToLocation(double tx, double ty, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack){
        telemetry.addData("x", tx);
        telemetry.addData("y", ty);

        //rightDead and leftDead should increase when moving forwards
        //perpDead should increase when moving right

        DcMotor leftDead = leftFront;
        DcMotor rightDead = rightFront;
        DcMotor perpDead = leftBack;

        double startTicksR = rightDead.getCurrentPosition();
        double startTicksP = perpDead.getCurrentPosition();

        double ticksPerInch = 336.8358584;

        double scaleFactor = 1.0;
        double scaleDistance = 30.0;
        double stopDistance = 9.0;

        double currentLocationX = 0;
        double currentLocationY = 0;

        boolean reachedLocation = false;

        while (!reachedLocation) {
            double distance = Math.sqrt(Math.pow(tx-currentLocationX,2)+Math.pow(ty-currentLocationY,2));
            telemetry.addData("distance from the target",distance);
            if(distance<=stopDistance) reachedLocation = true;
            if(distance>=scaleDistance) scaleFactor = 1.0;
            else{
                scaleFactor = distance/scaleDistance;
            }
            telemetry.addData("scale factor right now",scaleFactor);

            double moveY = ty - currentLocationY;
            double moveX = tx - currentLocationX;
            double bigNumber = Math.max(moveX,moveY);

            double rotationY = (moveY/bigNumber)*scaleFactor;
            double rotationX = (moveX/bigNumber)*scaleFactor;

            // calculate how much each motor should move`
            double flPower = (rotationY + rotationX);
            double frPower = (rotationY - rotationX);
            double blPower = (rotationY - rotationX);
            double brPower = (rotationY + rotationX);

            leftFront.setPower(flPower);
            rightFront.setPower(frPower);
            leftBack.setPower(blPower);
            rightBack.setPower(brPower);

            double changeTicksR = rightDead.getCurrentPosition() - startTicksR;
            double changeTicksP = perpDead.getCurrentPosition() - startTicksP;

            currentLocationX = changeTicksR / ticksPerInch;
            currentLocationY = changeTicksP / ticksPerInch;

            telemetry.addData("x coord", currentLocationX);
            telemetry.addData("y coord", currentLocationY);

            telemetry.update();
        }
    }

    public void armAngle(double angle, DcMotor leftM, DcMotor rightM){
        double gearRatio = 1.0/5.0;

        double currentTicks = leftM.getCurrentPosition();

        //the starting angle of the arm when the code is run
        double zeroTickAngle = -10;

        //how many ticks the encoder goes through for each motor rotation
        double tickPerRot = ((((1+(46/17))) * (1+(46/11))) * 28);

        //calculates the arm's current angle
        double startingAngle = (((currentTicks/tickPerRot)*gearRatio)*360) + zeroTickAngle;
        telemetry.addData("startingAngle", startingAngle);

        double angleDiff = angle - startingAngle;
        telemetry.addData("angleDiff", angleDiff);

        double rotationsToDo = (angleDiff/360)/gearRatio;
        telemetry.addData("rotationsToDo", rotationsToDo);

        int ticksToMove = (int) (currentTicks + (rotationsToDo * tickPerRot));

        telemetry.update();

        leftM.setTargetPosition(ticksToMove);
        rightM.setTargetPosition(ticksToMove);
    }

    public void spinFlywheel(double power, DcMotor flyL, DcMotor flyR) {
        flyL.setPower(power);
        flyR.setPower(power);
    }
}
