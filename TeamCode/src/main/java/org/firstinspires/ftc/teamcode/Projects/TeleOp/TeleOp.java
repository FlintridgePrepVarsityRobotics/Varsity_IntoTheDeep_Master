package org.firstinspires.ftc.teamcode.Projects.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

//@TeleOp(name = "TestTeleop")
public class TeleOp extends LinearOpMode {

    public HWMapBasic robot = new HWMapBasic();
        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);
            double speed = .9;



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
//             Teleop Code goes here         }
                //drive climb wrist gamepad1 else is gamepad2
                if (gamepad1.y == true) {
                    }
                }
                if (gamepad1.right_bumper == true)
                {robot.Wrist.setPosition(0);

                }
                if (gamepad1.left_bumper == true)
                {robot.Wrist.setPosition(1);

                }
                if (gamepad2.a == true)
                {robot.armLeft.setPosition(0);
                    robot.armRight.setPosition(1);

                }
                if (gamepad2.b == true)
                {robot.armLeft.setPosition(1);
                    robot.armRight.setPosition(0);

                }
                if (gamepad1.x == true)
                {

                }
                if (gamepad1.y == true)
                {

                }
                if (gamepad2.dpad_right == true)
                {robot.baseLeft.setPosition(1);
                    robot.baseRight.setPosition(0);

                }
                if (gamepad2.dpad_left == true)
                {robot.baseLeft.setPosition(0);
                    robot.baseRight.setPosition(1);

                }
                if (gamepad2.dpad_up == true)
                {robot.baseLeft.setPosition(0.5);
                    robot.baseRight.setPosition(0.5);

                }
                else if (gamepad1.dpad_down == true)
                {

                }

            }
        }
