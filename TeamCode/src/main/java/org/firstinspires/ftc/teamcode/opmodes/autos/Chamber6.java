package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.LineToLinearPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

import java.util.function.Supplier;

@Config
@Autonomous(name = "Chamber 6", group = "Autos")
public class Chamber6 extends AutoCommandBase{
    public static Pose2dHelperClass startPose = new Pose2dHelperClass();
    public static Pose2dHelperClass grabSpecPose = new Pose2dHelperClass();
    public static Pose2dHelperClass spec1Pose = new Pose2dHelperClass();
    public static Pose2dHelperClass spec2Pose = new Pose2dHelperClass();
    public static Pose2dHelperClass spec3Pose = new Pose2dHelperClass();
    public static Pose2dHelperClass spec4Pose = new Pose2dHelperClass();
    public static Pose2dHelperClass spec5Pose = new Pose2dHelperClass();
    public static Pose2dHelperClass spec6Pose = new Pose2dHelperClass();

    public static long delayToUpLift = 300;
    public static long hangToOpenClaw = 300;
    public static long pathToStow = 0;

    @Override
    public Pose2d getStartPose() {
        return drive.getPoseEstimate();
    }

    @Override
    public void initializeSuperStructure() {
        drive.setPoseEstimate(startPose.toPose2d());
        slide.foldSlideStructure();
        slide.backwardSlideExtension();
        liftClaw.grabFromWall();
        liftClaw.closeClaw();
    }

    @Override
    public Command runAutoCommand() {
        Supplier<Command> hangSpecimen =
                () -> new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG));

        Supplier<Command> stowLiftAndArm =
                () ->

                        new SequentialCommandGroup(liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 100),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));

        return new SequentialCommandGroup(
                new LineToLinearPathCommand(drive, spec1Pose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToPreHang())),
                hangSpecimen.get(),
                wait(drive, hangToOpenClaw),
                liftClaw.openClawCommand(),
                //TODO Sequence to give samples to hup


                //Last 4 samples
                liftClaw.closeClawCommand(),
                new LineToLinearPathCommand(drive, spec2Pose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToPreHang())),
                hangSpecimen.get(),
                wait(drive, hangToOpenClaw),
                liftClaw.openClawCommand(),

                new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
                liftClaw.closeClawCommand(),
                new LineToLinearPathCommand(drive, spec3Pose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToPreHang())),
                hangSpecimen.get(),
                wait(drive, hangToOpenClaw),
                liftClaw.openClawCommand(),

                new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
                liftClaw.closeClawCommand(),
                new LineToLinearPathCommand(drive, spec4Pose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToPreHang())),
                hangSpecimen.get(),
                wait(drive, hangToOpenClaw),
                liftClaw.openClawCommand(),

                new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
                liftClaw.closeClawCommand(),
                new LineToLinearPathCommand(drive, spec5Pose.toPose2d())
                        .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToPreHang())),
                hangSpecimen.get(),
                wait(drive, hangToOpenClaw),
                liftClaw.openClawCommand(),
                new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
                        .alongWith(stowLiftAndArm.get())
        );
    }
}
