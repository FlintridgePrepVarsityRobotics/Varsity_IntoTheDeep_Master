package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "TestTeleop")
public class TestTeleop extends LinearOpMode {

    public HWMapBasic robot = new HWMapBasic();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .9;
        int rightPosition = 0;
        int leftPosition = 0;
        int[] positions;

        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        boolean isSpinning = false;
        while (opModeIsActive()) {
            boolean aButtonHeld = false;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x*1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fLeftPower = (y + x + rx) / denominator;
            double bLeftPower = (y - x + rx) / denominator;
            double fRightPower = (y - x - rx) / denominator;
            double bRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(fLeftPower * speed);
            robot.bLeftWheel.setPower(bLeftPower * speed);
            robot.fRightWheel.setPower(fRightPower * speed);
            robot.bRightWheel.setPower(bRightPower * speed);

            if (gamepad1.left_trigger == 1&&rightPosition<0 && leftPosition < 0) {

                rightPosition += 5;
                leftPosition += 5;

                robot.rightLift.setPower(.4);
                robot.leftLift.setPower(.4);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();
            }
            else if (gamepad1.right_trigger == 1&&rightPosition > -1800 && leftPosition > -1800) {

                rightPosition -= 5;
                leftPosition -= 5;

                robot.rightLift.setPower(.4);
                robot.leftLift.setPower(.4);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();



            }
//             Teleop Code goes here         }
            //drive climb wrist gamepad1 else is gamepad2
            if (gamepad1.y == true) {
            }
        }
        if (gamepad1.right_bumper == true)
        {robot.wrist.setPosition(0);

        }
        if (gamepad1.left_bumper == true)
        {robot.wrist.setPosition(1);

        }
        if (gamepad2.a == true)
        {robot.lArm.setPosition(0);
            robot.rArm.setPosition(1);

        }
        if (gamepad2.b == true)
        {robot.lArm.setPosition(1);
            robot.rArm.setPosition(0);

        }
        if (gamepad1.x == true)
        {

        }
        if (gamepad1.y == true)
        {

        }
        if (gamepad2.dpad_right == true)
        {robot.lBar.setPosition(1);
            robot.rBar.setPosition(0);

        }
        if (gamepad2.dpad_left == true)
        {robot.lBar.setPosition(0);
            robot.rBar.setPosition(1);

        }
        if (gamepad2.dpad_up == true)
        {robot.lBar.setPosition(0.5);
            robot.rBar.setPosition(0.5);

        }
        else if (gamepad1.dpad_down == true)
        {

        }

    }

    int[] WaitTillTargetReached(int tolerance, boolean lock){
        int leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
        int rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());
        int check=102930293;
        while(leftDifference > tolerance || rightDifference > tolerance)

        {

            leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
            rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

            robot.leftLift.setPower(0.5);
            robot.rightLift.setPower(0.5);
            if (check == robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition()) {
                break;
            }
            else {
                check = robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition();
            }
            sleep(1);
            int a = robot.rightLift.getCurrentPosition();
            int c = robot.leftLift.getCurrentPosition();
            telemetry.addLine("current position: " + a + "," + c);
            telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
            telemetry.update();

        }
        int a = robot.rightLift.getCurrentPosition();
        int c = robot.leftLift.getCurrentPosition();
        telemetry.addLine("current position: " + a + "," + c);
        telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
        telemetry.update();
        int[] positions = new int[] {a,c};


        if(!lock)
        {
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);
        }
        return(positions);

    }
    private void cycle() {

    }
}
