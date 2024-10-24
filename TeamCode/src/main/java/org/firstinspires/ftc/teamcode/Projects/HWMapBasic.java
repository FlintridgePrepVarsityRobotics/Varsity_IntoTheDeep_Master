package org.firstinspires.ftc.teamcode.Projects;


import android.widget.Button;

import com.google.blocks.ftcrobotcontroller.util.ProjectsUtil;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.Project;



public class HWMapBasic extends Project {
    public DcMotor fLeftWheel = null;
    //2
    public DcMotor fRightWheel = null;
    //3
    public DcMotor bLeftWheel = null;
    //1
    public DcMotor bRightWheel = null;
    //0
    public DcMotor leftLift = null;


    public DcMotor rightLift = null;
    public Servo claw = null;

    public Servo wrist = null;

    public Servo lBar = null;

    public Servo rBar = null;

    public Servo lArm = null;

    public Servo rArm = null;






    public WebcamName camera = null;


    //@Override
    public void init(HardwareMap hwMap) {
        // Get motors from hardware map
        fLeftWheel = hwMap.dcMotor.get("FL");
        fRightWheel = hwMap.dcMotor.get("FR");
        bLeftWheel = hwMap.dcMotor.get("BL");
        bRightWheel = hwMap.dcMotor.get("BR");
        leftLift = hwMap.dcMotor.get("LL");
        rightLift = hwMap.dcMotor.get("RL");
        claw = hwMap.servo.get("Claw");
        wrist = hwMap.servo.get("Wrist");
        lBar = hwMap.servo.get("lBar");
        rBar = hwMap.servo.get("rBar");
        lArm = hwMap.servo.get("lArm");
        rArm = hwMap.servo.get("rArm");




//        oClaw = hwMap.servo.get("oClaw");
        //wrist = hwMap.servo.get("wrist");
        //wrist = hwMap.dcMotor.get("wrist");



        // Set Direction
        fRightWheel.setDirection(DcMotor.Direction.FORWARD);
        fLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        bRightWheel.setDirection(DcMotor.Direction.FORWARD);
        bLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        //wrist.setDirection(DcMotor.Direction.FORWARD);

        // Set run mode
        fRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set brakes
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get webcam from hardware map
        // camera = hwMap.get(WebcamName.class, "webcam");

        Stop();
    }
    public void Stop(){
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);
        claw.setPosition(0);
        wrist.setPosition(1);
        lArm.setPosition(0);
        rArm.setPosition(1);
        lBar.setPosition(0);
        rBar.setPosition(1);

//        slide.setPower(0);
        //wrist.setPower(0);
        //kevin-Pain
    }
}
