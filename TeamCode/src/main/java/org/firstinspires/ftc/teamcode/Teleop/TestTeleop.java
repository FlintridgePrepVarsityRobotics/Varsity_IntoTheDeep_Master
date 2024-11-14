package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "TestTeleop")
public class TestTeleop extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .8;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int[] positions;

        // init motor pos and encoder
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("" + robot.leftLift.getCurrentPosition());
        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        boolean isSpinning = false;
        while (opModeIsActive()) {
            boolean aButtonHeld = false;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x*1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x*.7;
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
//gamepad1=
            if (gamepad2.left_bumper&&leftPosition<0&&rightPosition<0) {

                rightPosition += 45;
                leftPosition += 45;

                robot.rightLift.setPower(.8);
                robot.leftLift.setPower(.8);

                // If one of them is greater than 0 (out of bound) then set both to zero
                if(rightPosition > 0 || leftPosition > 0){
                    rightPosition = 0;
                    leftPosition = 0;
                }

                // Run to pos
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);

                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();
            }
            else if (gamepad2.right_bumper&&leftPosition>-8000&&rightPosition>-8000) {

                rightPosition -= 40;
                leftPosition -= 40;

                robot.rightLift.setPower(.8);
                robot.leftLift.setPower(.8);
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
            if (gamepad1.a) {
                robot.claw.setPosition(0); //claw closed
            }
            else if (gamepad1.b){
                robot.claw.setPosition(0.315);
            }

            if (gamepad1.x) {
                robot.claw.setPosition(.415);
            }

            if (gamepad1.right_bumper)
            {robot.wrist.setPosition(0);

            }
            else if (gamepad1.left_bumper)
            {robot.wrist.setPosition(1);

            }
            else if (gamepad1.dpad_right) //wrist midpoint
            {robot.wrist.setPosition(0.5);
            }

            else if(gamepad1.dpad_left){ //specimen grab
                robot.wrist.setPosition(.73);
            }

            if (gamepad1.dpad_up){ //Wrist Up (when arm is flipped, specimen score, 1 or 0)
                robot.wrist.setPosition(.825);
            }

            if (gamepad1.dpad_down){ //Wrist Down (when arm is flipped, sample grab, 1 or 0)
                robot.wrist.setPosition(0);
            }
            if(gamepad2.y){
                robot.rightLift.setTargetPosition(-5015);
                robot.leftLift.setTargetPosition(-5015);
                positions = WaitTillTargetReached(50, true);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                rightPosition = positions[0];
                leftPosition = positions[1];
            }




            // TODO: INIT POS 0 FOR HEIGHT
            if (gamepad2.a){
                telemetry.addLine("Setting slider to zero");
                telemetry.update();
                robot.leftLift.setTargetPosition(0);
                robot.rightLift.setTargetPosition(0);

                positions = WaitTillTargetReached(50, true);
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                rightPosition = positions[0];
                leftPosition = positions[1];
            }

            if (gamepad2.y){
                //lift
                robot.leftLift.setTargetPosition(-8000);
                robot.rightLift.setTargetPosition(-8000);

                positions = WaitTillTargetReached(50, true);
//                robot.rightLift.setTargetPosition(rightPosition);
//                robot.leftLift.setTargetPosition(leftPosition);
//                rightPosition = positions[0];
//                leftPosition = positions[1];
                robot.rArm.setPosition(.9); //arm in 1
                robot.lArm.setPosition(.1); // 0
                robot.wrist.setPosition(0.5); //wrist midpoint
            }

            if (gamepad2.b)
            {
                robot.rArm.setPosition(.9); //arm in 1
                robot.lArm.setPosition(.1); // 0

                robot.rightLift.setTargetPosition(-5300);
                robot.leftLift.setTargetPosition(-5300);
                robot.rightLift.setPower(.8);
                robot.leftLift.setPower(.8);
                positions = WaitTillTargetReached(50, true);
//                robot.rightLift.setTargetPosition(rightPosition);
//                robot.leftLift.setTargetPosition(leftPosition);
//                rightPosition = positions[0];
//                leftPosition = positions[1];



                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();

                telemetry.addLine("Running button command");
                telemetry.addLine("current position: " + a + ", " + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();

                robot.wrist.setPosition(.825);
            }

            if (gamepad2.x){
                robot.rArm.setPosition(.9); //arm in 1
                robot.lArm.setPosition(.1); // 0
                telemetry.addLine("Setting slider to zero");
                robot.leftLift.setTargetPosition(0);
                robot.rightLift.setTargetPosition(0);
                robot.wrist.setPosition(.6);
            }


            if (gamepad2.dpad_right)
            {robot.rArm.setPosition(.375); //arm out 0
                robot.lArm.setPosition(.625); // 1
            }
            if (gamepad2.dpad_left)
            {robot.rArm.setPosition(.9); //arm in 1
                robot.lArm.setPosition(.1); // 0
            }
            if (gamepad2.dpad_up)
            {robot.lBar.setPosition(0.5);
                robot.rBar.setPosition(0.5);

            }
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

            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);
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
