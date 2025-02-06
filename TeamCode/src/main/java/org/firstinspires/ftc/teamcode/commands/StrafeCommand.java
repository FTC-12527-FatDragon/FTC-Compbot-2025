package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

@Config
public class StrafeCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final Vision vision;
    private final PIDController strafePID;
    private double targetToCameraTx = 0;
    private Pose2D initialPose;

    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0;
    public static double positionTolerance = 0;

    private final Telemetry telemetry;

    public StrafeCommand(SampleMecanumDrive drive, Vision vision, Telemetry telemetry) {
        this.drive = drive;
        this.vision = vision;
        this.telemetry = telemetry;
        strafePID = new PIDController(kP, kI, kD);
        strafePID.setTolerance(positionTolerance);
    }

    @Override
    public void initialize() {
        targetToCameraTx = vision.getStrafeOffset();
        initialPose = drive.od.getMetricPose();
    }

    @Override
    public void execute() {
        Pose2D currentPose = drive.od.getMetricPose();
        // Calculate the change in position relative to initial position in robot frame
        double deltaX = (currentPose.getX(DistanceUnit.MM) - initialPose.getX(DistanceUnit.MM)) * Math.cos(currentPose.getHeading(AngleUnit.RADIANS)) +
                (currentPose.getY(DistanceUnit.MM) - initialPose.getY(DistanceUnit.MM)) * Math.sin(currentPose.getHeading(AngleUnit.RADIANS));


        double error = targetToCameraTx - deltaX;


        // Calculate the strafe power using PID
        double strafePower = strafePID.calculate(-error, 0);

        telemetry.addData("target camera tx", targetToCameraTx);
        telemetry.addData("delta X", deltaX);
        telemetry.addData("Error", -error);
        telemetry.addData("Strafe Power", strafePower);
        telemetry.addData("Target Visible", vision.isTargetVisible());
        telemetry.update();


        // Apply the strafe movement (robot-relative)
        drive.setWeightedDrivePower(new Pose2d(0, strafePower, 0));
    }

    @Override
    public boolean isFinished() {
        /*return !vision.isTargetVisible() || strafePID.atSetPoint();*/
        return false;
    }
}
