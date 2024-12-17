package frc.robot.auto;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";

    private static final double TranslationToleranceMeters = 0.5;
    private static final double TimeToleranceSeconds = 0.1;

    private final Swerve swerve;
    private final PhotonVision photonVision;
    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;

        this.autoFactory = Choreo.createAutoFactory(
            swerve::getPose,
            swerve::followChoreoSample,
            Robot.IsRedAlliance,
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

    private Command resetPose(final AutoTrajectory swerveSample) {
        return Commands.defer(() ->
                photonVision.resetPoseCommand(swerveSample.getInitialPose().orElse(swerve.getPose())),
                Set.of()
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.running().whileTrue(
                Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled())
        );

        return routine;
    }

    public AutoRoutine squigleAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("Squigle");
        final AutoTrajectory simpleSquigle = routine.trajectory("SimpleSquigle");

        routine.running().whileTrue(
                Commands.sequence(
                        resetPose(simpleSquigle),
                        squigleAuto().cmd()
                )
        );

        simpleSquigle.atTime(0.33).onTrue(
                Commands.print("REACHED MARKER")
        );

        return routine;
    }

    public EventLoop followNote() {
        final EventLoop eventLoop = new EventLoop();
        final Trigger trigger = new Trigger(eventLoop, DriverStation::isAutonomousEnabled);

        trigger.whileTrue(
                Commands.repeatingSequence(
                        swerve.driveToOptionalPose(() -> photonVision.getBestNotePose(swerve::getPose))
                )
        );

        return eventLoop;
    }
}
