package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@Autonomous(name = "BeyondAuto")
public class BeyondAuto extends LinearOpMode{


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
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("" + robot.leftLift.getCurrentPosition());
        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(!isStarted()){




        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here
        sleep(9000);
        robot.fRightWheel.setPower(.25);
        robot.fLeftWheel.setPower(-.25);
        robot.bRightWheel.setPower(-.25);
        robot.bLeftWheel.setPower(.25);
        sleep(1000);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);
        robot.fRightWheel.setPower(.25);
        robot.fLeftWheel.setPower(.25);
        robot.bRightWheel.setPower(.25);
        robot.bLeftWheel.setPower(.25);
        sleep(750);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(500);

        robot.rightLift.setTargetPosition(-6000);
        robot.leftLift.setTargetPosition(-6000);
        positions = WaitTillTargetReached(50, true);
//        robot.rightLift.setTargetPosition(rightPosition);
//        robot.leftLift.setTargetPosition(leftPosition);
//        rightPosition = positions[0];
//        leftPosition = positions[1];
        sleep(2500);
        robot.fRightWheel.setPower(.25);
        robot.fLeftWheel.setPower(.25);
        robot.bRightWheel.setPower(.25);
        robot.bLeftWheel.setPower(.25);
        sleep(500);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(1000);

        robot.rightLift.setTargetPosition(-5600);
        robot.leftLift.setTargetPosition(-5600);
        positions = WaitTillTargetReached(50, true);
//        robot.rightLift.setTargetPosition(rightPosition);
//        robot.leftLift.setTargetPosition(leftPosition);
//        rightPosition = positions[0];
//        leftPosition = positions[1];
        sleep(1000);
        robot.fRightWheel.setPower(-.25);
        robot.fLeftWheel.setPower(-.25);
        robot.bRightWheel.setPower(-.25);
        robot.bLeftWheel.setPower(-.25);
        sleep(650);
        robot.claw.setPosition(1);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        sleep(1000);
        robot.fRightWheel.setPower(-.25);
        robot.fLeftWheel.setPower(-.25);
        robot.bRightWheel.setPower(-.25);
        robot.bLeftWheel.setPower(-.25);
        sleep(500);
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        positions = WaitTillTargetReached(50, true);
        robot.rightLift.setTargetPosition(rightPosition);
        robot.leftLift.setTargetPosition(leftPosition);
        rightPosition = positions[0];
        leftPosition = positions[1];


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}



    }
    public void preset1 (){






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

        //specimen scoring

//if (gamepad2.) {
//    robot.rArm.setPosition(0);       //Extend fourbar, extend arm, move wrist to face downward, open claw
//    robot.lArm.setPosition(1);
//    robot.wrist.setPosition(.25);
//    robot.claw.setPosition(.25);
//}
//
//if (gamepad2.) {
//    robot.rBar.setPosition(1);
//    robot.lBar.setPosition(0);
//    robot.rArm.setPosition(1);      //Retract fourbar, retract arm, have claw neutral(same direction as arm)
//    robot.lArm.setPosition(0);
//    robot.claw.setPosition(.5);
//}
//
//if (gamepad2.) {
//    robot.rBar.setPosition(1);
//    robot.lBar.setPosition(0);
//    robot.rArm.setPosition(1);       //retract fourbar, retract arm, have claw face slightly toward the ground
//    robot.lArm.setPosition(0);
//    robot.wrist.setPosition(1);
//}

//if (gamepad2.) {
//    robot.rBar.setPosition();
//    robot.lBar.setPosition();       //preset for pickup from wall from human player
//    robot.rArm.setPosition();       //retract fourbar, retract arm, wrist tilt slightly toward ground, lift rise to specimen height, then lift rises again after grabbing
//    robot.lArm.setPosition();
//    robot.wrist.setPosition();
//    robot.leftLift.setPower();
//    robot.rightLift.setPower();
//    robot.claw.setPosition();
//    robot.leftLift.setPower();
//    robot.rightLift.setPower();
    }
}


