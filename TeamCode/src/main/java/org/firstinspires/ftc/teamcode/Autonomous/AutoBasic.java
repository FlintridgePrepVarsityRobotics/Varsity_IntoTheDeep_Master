package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@Autonomous(name = "AutoBasic")
public class AutoBasic extends LinearOpMode{


    public HWMapBasic robot = new HWMapBasic();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);


        int direction = 1;
        int otherDirection = -1;
        boolean isRight = true;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int[] positions;


        while(!isStarted()){




        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here
        robot.fRightWheel.setPower(.5);
        robot.fLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        sleep(1500);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);

        robot.rightLift.setTargetPosition(-6000);
        robot.leftLift.setTargetPosition(-6000);
        positions = WaitTillTargetReached(50, true);
        robot.rightLift.setTargetPosition(rightPosition);
        robot.leftLift.setTargetPosition(leftPosition);
        rightPosition = positions[0];
        leftPosition = positions[1];

        robot.fRightWheel.setPower(.5);
        robot.fLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        sleep(500);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);

        robot.rightLift.setTargetPosition(-5500);
        robot.leftLift.setTargetPosition(-5500);
        positions = WaitTillTargetReached(50, true);
        robot.rightLift.setTargetPosition(rightPosition);
        robot.leftLift.setTargetPosition(leftPosition);
        rightPosition = positions[0];
        leftPosition = positions[1];
        sleep(100);
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.5);
        sleep(300);
        robot.claw.setPosition(1);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);
        robot.fRightWheel.setPower(-.5);
        robot.fLeftWheel.setPower(-.5);
        robot.bRightWheel.setPower(-.5);
        robot.bLeftWheel.setPower(-.5);
        sleep(300);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}



    }
    public void preset1 (){


        robot.rBar.setPosition(0.5);
        robot.lBar.setPosition(0.5);

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


