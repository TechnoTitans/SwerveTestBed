package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;

        this.autoFactory = new AutoFactory(
                swerve::getPose,
                photonVision::resetPose,
                swerve::followChoreoSample,
                true,
                swerve,
                (trajectory, trajectoryRunning) -> {
                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Path",
                            (Robot.IsRedAlliance.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses()
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Name",
                            trajectory.name()
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Running",
                            trajectoryRunning
                    );
                }
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );

        return routine;
    }

    public AutoRoutine straight() {
        final AutoRoutine routine = autoFactory.newRoutine("straight");
        final AutoTrajectory straight = routine.trajectory("Straight");

        routine.active().onTrue(
                Commands.sequence(
                    straight.resetOdometry(),
                    straight.cmd()
                )
        );

        straight.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }
}
