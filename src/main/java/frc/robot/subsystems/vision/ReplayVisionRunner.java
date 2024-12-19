package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionPoseEstimator;
import frc.robot.subsystems.vision.estimator.VisionUpdate;
import frc.robot.subsystems.vision.result.NoteTrackingResult;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;

public class ReplayVisionRunner implements PhotonVisionRunner {
    public static class VisionIOReplay implements VisionIO {
        private final PhotonCamera photonCamera;
        private final Transform3d robotToCamera;

        public VisionIOReplay(final TitanCamera titanCamera) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
        }
    }

    final AprilTagFieldLayout aprilTagFieldLayout;

    private final Map<VisionIOReplay, String> visionIONames;
    private final Map<VisionIOReplay, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<VisionIOReplay, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap;

    private final Map<VisionIO, VisionUpdate> visionUpdates;
    private final Map<VisionIO, NoteTrackingResult> noteTrackingResultMap;

    public ReplayVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<VisionIOReplay, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<VisionIOReplay, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap
    ) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;

        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.noteTrackingVisionIOInputsMap = noteTrackingVisionIOInputsMap;

        final Map<VisionIOReplay, String> visionIONames = new HashMap<>();
        for (final VisionIOReplay visionIOApriltagsReplay : apriltagVisionIOInputsMap.keySet()) {
            visionIONames.put(visionIOApriltagsReplay, visionIOApriltagsReplay.photonCamera.getName());
        }

        for (final VisionIOReplay visionIONoteTrackReplay : noteTrackingVisionIOInputsMap.keySet()) {
            visionIONames.put(visionIONoteTrackReplay, visionIONoteTrackReplay.photonCamera.getName());
        }

        this.visionIONames = visionIONames;
        this.visionUpdates = new HashMap<>();
        this.noteTrackingResultMap = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic(final Pose2d currentRobotPose) {
        if (ToClose.hasClosed()) {
            return;
        }

        for (
                final Map.Entry<VisionIOReplay, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOReplay visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            for (final PhotonPipelineResult result : pipelineResults) {
                VisionPoseEstimator.update(
                        aprilTagFieldLayout,
                        currentRobotPose,
                        visionIO.robotToCamera,
                        result
                ).ifPresent(
                        visionUpdate -> visionUpdates.put(visionIO, visionUpdate)
                );
            }
        }

        for (
                final Map.Entry<VisionIOReplay, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : noteTrackingVisionIOInputsMap.entrySet()
        ) {
            final VisionIOReplay visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            final PhotonPipelineResult pipelineResult = pipelineResults[pipelineResults.length - 1];
            Logger.recordOutput(
                    String.format("%s/%s/HasTarget", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    pipelineResult.hasTargets()
            );

            final NoteTrackingResult noteTrackingResult = new NoteTrackingResult(inputs.robotToCamera, pipelineResult);
            noteTrackingResultMap.put(visionIO, noteTrackingResult);
        }
    }

    /**
     * Reset the simulated robot {@link Pose3d}.
     * @param robotPose the new robot {@link Pose3d}
     */
    @Override
    public void resetRobotPose(final Pose3d robotPose) {}

    @Override
    public Map<VisionIOReplay, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<VisionIOReplay, VisionIO.VisionIOInputs> getNoteTrackingVisionIOInputsMap() {
        return noteTrackingVisionIOInputsMap;
    }

    @Override
    public VisionUpdate getVisionUpdate(final VisionIO visionIO) {
        return visionUpdates.get(visionIO);
    }

    @Override
    public NoteTrackingResult getNoteTrackingResult(final VisionIO visionIO) {
        return noteTrackingResultMap.get(visionIO);
    }
}
