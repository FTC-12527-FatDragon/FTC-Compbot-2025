package org.firstinspires.ftc.teamcode.opmodes;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@TeleOp(name = "TXTeleop")
public class TXBetaBot extends CommandOpMode {
    private GamepadEx gamepadEx1;
    private Lift lift;
    private LiftClaw liftClaw;
    private SlideSuperStucture slide;
    private MecanumDrive drive;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);

        lift = new Lift(hardwareMap, telemetry);
        liftClaw = new LiftClaw(hardwareMap);
        slide = new SlideSuperStucture(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap);

        //Teleop Drive Command
        drive.setDefaultCommand(new TeleopDriveCommand(
                drive, () -> -gamepadEx1.getLeftY(),
                () -> -gamepadEx1.getLeftX(), () -> gamepadEx1.getRightX(),
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
        ));


        //Basket Up Command
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.BASKET)),
                        new WaitUntilCommand(() -> lift.getCurrentPosition() > 600)
                                .andThen(new InstantCommand(liftClaw::upLiftArm))
                )
        );


        //Basket Drop and Back
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new InstantCommand(() -> slide.slideArmDown())
                                        .andThen(new WaitCommand(100))
                                        .andThen(new InstantCommand(() -> slide.setGoal(SlideSuperStucture.Goal.AIM))),
                                new InstantCommand(),
                                () -> lift.getGoal() == Lift.Goal.HANG
                        ),
                        new InstantCommand(liftClaw::openClaw),
                        new WaitCommand(100),
                        new InstantCommand(liftClaw::foldLiftArm),
                        new WaitCommand(500),
                        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW))
                )
        );

        //Aim
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                slide.aimCommand(), false
        );

        //Grab when aim
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)
                        && slide.getGoal() == SlideSuperStucture.Goal.AIM)
                .whenPressed(slide.grabCommand(), false);

        // Pure Handoff
//        Command handoffCommand =

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                        && slide.getGoal() == SlideSuperStucture.Goal.AIM)
                .whenPressed(slide.handoffCommand()
                        .alongWith(new InstantCommand(liftClaw::openClaw))
//                        .alongWith(new InstantCommand(() -> slide.setNormalHandoff(true)))
                        .andThen(new WaitCommand(600))
                        .andThen(new InstantCommand(() -> liftClaw.closeClaw()))
                        .andThen(new WaitCommand(300))
                        .andThen(new InstantCommand(() -> slide.openIntakeClaw())), false);

        //Handoff from Aim
        //Chamber Command
        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                && slide.getGoal() == SlideSuperStucture.Goal.AIM)
                .whenPressed(
                        slide.handoffCommand()
                                .alongWith(new InstantCommand(liftClaw::openClaw))
//                                .alongWith(new InstantCommand(() -> slide.setNormalHandoff(true)))
                                .andThen(new WaitCommand(600))
                                .andThen(new InstantCommand(() -> liftClaw.closeClaw()))
                                .andThen(new WaitCommand(300))
                                .andThen(new InstantCommand(() -> slide.openIntakeClaw()))
                                .andThen(new WaitCommand(200))
                                .andThen(new InstantCommand(() -> slide.wristUp()))
                                .andThen(new WaitCommand(200))
                                .andThen(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
                                                new WaitUntilCommand(() -> lift.getCurrentPosition() > 200)
                                                        .andThen(new InstantCommand(liftClaw::upLiftArm)))), false);

        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                && lift.getGoal() == Lift.Goal.HANG)
                .whenPressed(new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)));

        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                && slide.getGoal() == SlideSuperStucture.Goal.HANDOFF)
                .whenPressed(
                        new InstantCommand(() -> slide.wristUp())
                                .andThen(new WaitCommand(200))
                                .andThen(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
                                                new WaitUntilCommand(() -> lift.getCurrentPosition() > 200)
                                                        .andThen(new InstantCommand(liftClaw::upLiftArm)))),false
                );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) && lift.getGoal() == Lift.Goal.PRE_HANG)
                .whenPressed(new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG)));

        new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                                && lift.getGoal() == Lift.Goal.STOW)
                .whenPressed(lift.resetCommand());


        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                        && slide.getGoal() != SlideSuperStucture.Goal.HANDOFF)
                .whenPressed(new InstantCommand(slide::forwardSlideExtension));

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                        && slide.getGoal() != SlideSuperStucture.Goal.HANDOFF)
                .whenPressed(new InstantCommand(slide::backwardSlideExtension));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> slide.leftTurnServo())
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> slide.rightTurnServo())
        );

    }


}
