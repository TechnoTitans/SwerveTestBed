package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";
    private static final double NoteSearchEndingToleranceMeters = 0.25;

    private final Swerve swerve;
    private final PhotonVision photonVision;
    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;

        this.autoFactory = new AutoFactory(
            swerve::getPose,
            photonVision::resetPosition,
            swerve::followChoreoSample,
            true,
            swerve,
            new AutoFactory.AutoBindings(),
            (trajectory, trajectoryStarting) -> {
                Logger.recordOutput(
                    Autos.LogKey + "/Trajectory",
                    trajectory.getPoses()
                );

                Logger.recordOutput(
                    Autos.LogKey + "/TrajectoryStarting",
                    trajectoryStarting
                );
            }
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled())
        );

        return routine;
    }

    public AutoRoutine squiggleAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Squiggle");
        final AutoTrajectory simpleSquiggle = routine.trajectory("SimpleSquiggle");

        routine.active().whileTrue(
                Commands.sequence(
                        routine.resetOdometry(simpleSquiggle),
                        simpleSquiggle.cmd()
                )
        );

        simpleSquiggle.atTime(0.33).onTrue(
                Commands.print("REACHED MARKER")
        );

        simpleSquiggle.done().onTrue(swerve.stopCommand());

        return routine;
    }

    public AutoRoutine forwardAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Forward");
        final AutoTrajectory forwardTraj = routine.trajectory("Forward");

        routine.active().whileTrue(
                Commands.sequence(
                        routine.resetOdometry(forwardTraj),
                        forwardTraj.cmd()
                )
        );

        forwardTraj.done().onTrue(swerve.stopCommand());

        return routine;
    }

    private Command driveToNextNoteDumb(
            final Trigger hasNote,
            final Supplier<Pose2d> finishAtPose,
            final List<AutoTrajectory> returnTrajectories
    ) {
        return Commands.deadline(
                Commands.waitUntil(
                        () -> swerve.getPose()
                                .getTranslation()
                                .getDistance(
                                        finishAtPose.get().getTranslation()
                                ) <= Autos.NoteSearchEndingToleranceMeters
                ).andThen(swerve.stopCommand().asProxy()),
                Commands.sequence(
                        swerve.holdAxisFacingAngleAndDrive(
                                FieldConstants.FIELD_LENGTH_X_METERS/2,
                                Swerve.DriveAxis.X,
                                2,
                                finishAtPose
                        ).until(hasNote),
                        Commands.defer(() -> {
                            final Pose2d currentPose = swerve.getPose();
                            double closestDistance = Double.MAX_VALUE;
                            AutoTrajectory closestTrajectory = null;

                            if (returnTrajectories.isEmpty()) {
                                return Commands.none();
                            }

                            for (final AutoTrajectory trajectory : returnTrajectories) {
                                final Pose2d initialPose = trajectory.getInitialPose().orElseThrow();
                                final double distance = initialPose
                                        .getTranslation()
                                        .getDistance(currentPose.getTranslation());

                                if (distance < closestDistance || closestTrajectory == null) {
                                    closestDistance = distance;
                                    closestTrajectory = trajectory;
                                }
                            }

                            return closestTrajectory.cmd();
                        }, Set.of(swerve))
                )
        );
    }

    public AutoRoutine multiPieceNoPreload() {
        final AutoRoutine routine = autoFactory.newRoutine("MultiPiece");
        final AutoTrajectory startFlatToC0 = routine.trajectory("StartFlatToC0");
        final AutoTrajectory c0ToShootSource = routine.trajectory("C0ToShootSource");
        final AutoTrajectory shootSourceToC1 = routine.trajectory("ShootSourceToC1");
        final AutoTrajectory c1ToShootSource = routine.trajectory("C1ToShootSource");
        final AutoTrajectory shootSourceToC2 = routine.trajectory("ShootSourceToC2");
        final AutoTrajectory c2ToShootSource2 = routine.trajectory("C2ToShootSource2");
        final AutoTrajectory shootSource2ToPreload = routine.trajectory("ShootSource2ToPreload");
        final AutoTrajectory preloadToShootPreload = routine.trajectory("PreloadToShootPreload");

        final Trigger hasNote = routine.observe(NoteState.hasNote);
        hasNote.onFalse(Commands.waitSeconds(4).andThen(NoteState.setHasNoteCommand(true)));

        routine.active().onTrue(
                Commands.sequence(
                        routine.resetOdometry(startFlatToC0),
                        startFlatToC0.cmd()
                )
        );

        final Trigger atC0 = startFlatToC0.done();
        atC0.and(hasNote).onTrue(c0ToShootSource.cmd());
        atC0.and(hasNote.negate()).onTrue(driveToNextNoteDumb(
                hasNote,
                () -> FieldConstants.AMP_AUTO_NOTE_SEARCH_ENDING_POSE,
                List.of(c0ToShootSource, c1ToShootSource, c2ToShootSource2)
        ));

        c0ToShootSource.done().onTrue(
                Commands.print("Shooting")
                        .andThen(NoteState.setHasNoteCommand(false))
                        .andThen(shootSourceToC1.cmd())
        );

        final Trigger atC1 = shootSourceToC1.done();
        atC1.and(hasNote).onTrue(c1ToShootSource.cmd());
        atC1.and(hasNote.negate()).onTrue(driveToNextNoteDumb(
                hasNote,
                () -> FieldConstants.AMP_AUTO_NOTE_SEARCH_ENDING_POSE,
                List.of(c1ToShootSource, c2ToShootSource2)
        ));

        c1ToShootSource.done().onTrue(
                Commands.print("Shooting")
                        .andThen(shootSourceToC2.cmd())
        );

        final Trigger atC2 = shootSourceToC2.done();
        atC2.and(hasNote).onTrue(c2ToShootSource2.cmd());
        atC2.and(hasNote.negate()).onTrue(driveToNextNoteDumb(
                hasNote,
                () -> FieldConstants.AMP_AUTO_NOTE_SEARCH_ENDING_POSE,
                List.of(c2ToShootSource2)
        ));

        c2ToShootSource2.done().onTrue(Commands.print("Shooting").andThen(shootSource2ToPreload.cmd()));
        shootSource2ToPreload.done().and(hasNote).onTrue(preloadToShootPreload.cmd());

        preloadToShootPreload.done().onTrue(Commands.print("Shooting").andThen(swerve.stopCommand()));

        return routine;
    }
}
