package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto", group="Robot")
public class Auto extends LinearOpMode {
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.

        robot.init();
        waitForStart();
        //robot.AutoDrive(-1.5,0);

        robot.AutoStrafe(37,180);



    }
}
