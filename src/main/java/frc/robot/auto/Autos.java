package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.PhotonVision;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";

    private static final double TranslationToleranceMeters = 0.5;
    private static final double TimeToleranceSeconds = 0.1;

    private final Swerve swerve;
    private final PhotonVision photonVision;

    public Autos(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;
    }

    private static class AutoTriggers {
        private final ChoreoTrajectory trajectory;
        private final List<ChoreoTrajectory> trajectories;
        private final Supplier<Pose2d> poseSupplier;
        private final DoubleSupplier timeSupplier;
        private final EventLoop eventLoop;

        public AutoTriggers(
                final ChoreoTrajectory trajectory,
                final List<ChoreoTrajectory> trajectories,
                final Supplier<Pose2d> poseSupplier,
                final DoubleSupplier timeSupplier
        ) {
            this.trajectory = trajectory;
            this.trajectories = trajectories;
            this.poseSupplier = poseSupplier;
            this.timeSupplier = timeSupplier;
            this.eventLoop = new EventLoop();
        }

        public AutoTriggers(
                final String trajectoryName,
                final Supplier<Pose2d> poseSupplier,
                final DoubleSupplier timeSupplier
        ) {
            this(
                    Choreo.getTrajectory(trajectoryName),
                    Choreo.getTrajectoryGroup(trajectoryName),
                    poseSupplier,
                    timeSupplier
            );
        }

        public Trigger autoEnabled() {
            return new Trigger(eventLoop, DriverStation::isAutonomousEnabled);
        }

        // TODO: doesn't seem to ever trigger, also, theres probably? a better way to do this check
        @SuppressWarnings("unused")
        public Trigger atPlaceAndTime(final double timeSeconds) {
            final Translation2d place = trajectory
                    .sample(timeSeconds, Robot.IsRedAlliance.getAsBoolean())
                    .getPose()
                    .getTranslation();
            return new Trigger(
                    eventLoop,
                    () -> poseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(place) < TranslationToleranceMeters
                            && MathUtil.isNear(timeSeconds, timeSupplier.getAsDouble(), TimeToleranceSeconds)
            );
        }

        public Trigger atTime(final double timeSeconds) {
            return new Trigger(
                    eventLoop,
                    () -> MathUtil.isNear(timeSeconds, timeSupplier.getAsDouble(), TimeToleranceSeconds)
            );
        }

        // TODO: doesn't seem to ever trigger, also, theres probably? a better way to do this check
        public Trigger atPlace(final double timeSeconds) {
            final Translation2d place = trajectory
                    .sample(timeSeconds, Robot.IsRedAlliance.getAsBoolean())
                    .getPose()
                    .getTranslation();
            return new Trigger(
                    eventLoop,
                    () -> poseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(place) < TranslationToleranceMeters
            );
        }
    }

    private Command followPath(final ChoreoTrajectory choreoTrajectory) {
        return swerve.followChoreoPathCommand(choreoTrajectory, Robot.IsRedAlliance);
    }

    private Command followPath(final ChoreoTrajectory choreoTrajectory, final Timer timer) {
        return Commands.runOnce(timer::start)
                .andThen(followPath(choreoTrajectory))
                .finallyDo(timer::stop);
    }

    private Command resetPose(final ChoreoTrajectory trajectory) {
        return Commands.defer(() -> swerve.resetPoseCommand(
                        Robot.IsRedAlliance.getAsBoolean()
                                ? trajectory.getFlippedInitialPose()
                                : trajectory.getInitialPose()
                ),
                Set.of()
        );
    }

    public EventLoop doNothing() {
        final EventLoop doNothingEventLoop = new EventLoop();
        new Trigger(doNothingEventLoop, DriverStation::isAutonomousEnabled)
                .whileTrue(Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled()));

        return doNothingEventLoop;
    }

    public EventLoop squigleAuto () {
        final String trajectoryName = "Squigle";
        final Timer timer = new Timer();
        final AutoTriggers autoTriggers = new AutoTriggers(trajectoryName, swerve::getPose, timer::get);

//        autoTriggers.autoEnabled().whileTrue();

        autoTriggers.atTime(0.88).onTrue(
                Commands.print("REACHED MARKER")
        );

        return autoTriggers.eventLoop;
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
