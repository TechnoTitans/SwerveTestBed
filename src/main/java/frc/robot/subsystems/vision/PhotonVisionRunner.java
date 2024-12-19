package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.estimator.VisionUpdate;
import frc.robot.subsystems.vision.result.NoteTrackingResult;

import java.util.Map;

public interface PhotonVisionRunner {
    default void periodic(final Pose2d currentRobotPose) {}
    default void resetRobotPose(final Pose3d pose3d) {}

    default Map<? extends VisionIO, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return Map.of();
    }

    default Map<? extends VisionIO, VisionIO.VisionIOInputs> getNoteTrackingVisionIOInputsMap() {
        return Map.of();
    }

    default VisionUpdate getVisionUpdate(final VisionIO visionIO) {
        return null;
    }

    default NoteTrackingResult getNoteTrackingResult(final VisionIO visionIO) {
        return null;
    }
}
