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

        while(!isStarted()){




        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}



    }
    public void preset1 (){


        robot.rBar.setPosition(0.5);
        robot.lBar.setPosition(0.5);

    }
}


