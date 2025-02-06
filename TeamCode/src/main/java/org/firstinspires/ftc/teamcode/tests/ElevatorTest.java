package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.wpi.first.math.system.plant.DCMotor;

@Config
@TeleOp(name = "Elevator Test")
public class ElevatorTest extends LinearOpMode {
    public static double power = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Motor upLift = new Motor(hardwareMap, "liftMotorUp");
        Motor downLift = new Motor(hardwareMap, "liftMotorDown");
        upLift.resetEncoder();
        upLift.setInverted(true);
        downLift.setInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            upLift.set(power);
            downLift.set(power);
            telemetry.addData("Encoder Position", upLift.encoder.getPosition());
            telemetry.update();
        }
    }
}
