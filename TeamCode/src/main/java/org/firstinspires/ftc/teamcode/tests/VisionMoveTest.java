package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.StrafeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
@TeleOp(name = "Vision Move Test")
public class VisionMoveTest extends CommandOpMode {
    private GamepadEx gamepadEx1;
    private SampleMecanumDrive drive;
    private Vision limelight;

    public static double ledPWM = 0.5;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        limelight = new Vision(hardwareMap, telemetry);
        limelight.initializeCamera();
        drive = new SampleMecanumDrive(hardwareMap);

        limelight.setLEDPWM(ledPWM);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenHeld(new StrafeCommand(drive, limelight, telemetry)).whenReleased(() -> drive.setWeightedDrivePower(new Pose2d(0, 0, 0)));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenHeld(new StartEndCommand(() -> drive.setWeightedDrivePower(new Pose2d(0, 1, 0)), () -> drive.setWeightedDrivePower(new Pose2d(0,0,0))));

    }
}
