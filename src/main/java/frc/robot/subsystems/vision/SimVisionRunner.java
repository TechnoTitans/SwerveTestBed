package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionPoseEstimator;
import frc.robot.subsystems.vision.estimator.VisionUpdate;
import frc.robot.subsystems.vision.result.NoteTrackingResult;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;

public class SimVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagsSim implements VisionIO {
        public final PhotonCamera photonCamera;
        public final String cameraName;

        public final double stdDevFactor;
        public final Transform3d robotToCamera;

        public VisionIOApriltagsSim(
                final TitanCamera titanCamera,
                final VisionSystemSim visionSystemSim
        ) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();

            final PhotonCameraSim photonCameraSim =
                    new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

            photonCameraSim.enableDrawWireframe(true);
            photonCameraSim.enableRawStream(true);
            photonCameraSim.enableProcessedStream(true);

            ToClose.add(photonCameraSim);
            visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotToCameraTransform());
        }

        @Override
        public void updateInputs(final VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = stdDevFactor;
            inputs.robotToCamera = robotToCamera;
            inputs.pipelineResults = photonCamera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
        }
    }

    public static class VisionIONoteTrackingSim implements VisionIO {
        public final TitanCamera titanCamera;
        public final PhotonCamera photonCamera;
        public final String cameraName;

        public final Transform3d robotToCamera;

        public VisionIONoteTrackingSim(
                final TitanCamera titanCamera,
                final VisionSystemSim visionSystemSim
        ) {
            this.titanCamera = titanCamera;
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.robotToCamera = titanCamera.getRobotToCameraTransform();

            final PhotonCameraSim photonCameraSim =
                    new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

            photonCameraSim.enableDrawWireframe(true);
            photonCameraSim.enableRawStream(true);
            photonCameraSim.enableProcessedStream(true);

            ToClose.add(photonCameraSim);
            visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotToCameraTransform());
        }

        @Override
        public void updateInputs(final VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = -1;
            inputs.robotToCamera = robotToCamera;
            inputs.pipelineResults = photonCamera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
        }
    }

    private final Swerve swerve;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final VisionSystemSim visionSystemSim;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<VisionIONoteTrackingSim, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap;

    private final Map<VisionIO, VisionUpdate> visionUpdates;
    private final Map<VisionIO, NoteTrackingResult> noteTrackingResultMap;

    public SimVisionRunner(
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final AprilTagFieldLayout aprilTagFieldLayout,
            final VisionSystemSim visionSystemSim,
            final Pose2d[] simNotePoses,
            final Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<VisionIONoteTrackingSim, VisionIO.VisionIOInputs> noteTrackingVisionIOInputsMap
    ) {
        this.swerve = swerve;
        this.visionIndependentOdometry = visionIndependentOdometry;
        this.visionSystemSim = visionSystemSim;
        this.visionSystemSim.addAprilTags(aprilTagFieldLayout);

        this.aprilTagFieldLayout = aprilTagFieldLayout;

        for (final Pose2d simNotePose : simNotePoses) {
            this.visionSystemSim.addVisionTargets("note", new VisionTargetSim(
                    PoseUtils.note2dTo3d(simNotePose), SimConstants.Vision.NOTE_TARGET_MODEL
            ));
        }

        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.noteTrackingVisionIOInputsMap = noteTrackingVisionIOInputsMap;

        this.visionUpdates = new HashMap<>();
        this.noteTrackingResultMap = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic(final Pose2d currentRobotPose) {
        if (ToClose.hasClosed()) {
            return;
        }

        final Pose2d visionIndependentPose =
                visionIndependentOdometry.update(swerve.getYaw(), swerve.getModulePositions());

        visionSystemSim.update(
                GyroUtils.robotPose2dToPose3dWithGyro(
                        visionIndependentPose,
                        new Rotation3d(
                                swerve.getRoll().getRadians(),
                                swerve.getPitch().getRadians(),
                                swerve.getYaw().getRadians()
                        )
                )
        );

        for (
                final Map.Entry<VisionIOApriltagsSim, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOApriltagsSim visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            for (final PhotonPipelineResult result : pipelineResults) {
                VisionPoseEstimator.update(
                        inputs.name,
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
                final Map.Entry<VisionIONoteTrackingSim, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : noteTrackingVisionIOInputsMap.entrySet()
        ) {
            final VisionIONoteTrackingSim visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            final PhotonPipelineResult pipelineResult = pipelineResults[pipelineResults.length - 1];
            Logger.recordOutput(
                    String.format("%s/%s/HasTarget", PhotonVision.PhotonLogKey, inputs.name),
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
    public void resetRobotPose(final Pose3d robotPose) {
        final Pose2d currentPose = robotPose.toPose2d();

        visionIndependentOdometry.resetPosition(
                currentPose.getRotation(), swerve.getModulePositions(), currentPose
        );
        visionSystemSim.resetRobotPose(robotPose);
    }

    @Override
    public Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<VisionIONoteTrackingSim, VisionIO.VisionIOInputs> getNoteTrackingVisionIOInputsMap() {
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
