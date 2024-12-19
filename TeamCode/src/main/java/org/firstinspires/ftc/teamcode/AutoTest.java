package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="AutoTest", group="Test")
public class AutoTest extends LinearOpMode {
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.
        robot.init();
        waitForStart();
        telemetry.addData("Initial Heading", robot.getHeading());
        telemetry.update();

        // Autonomous drive
        robot.AutoDrive(60, 0);

        // Add telemetry to show final heading after drive
        telemetry.addData("Final Heading", robot.getHeading());
        telemetry.update();


    }
}
