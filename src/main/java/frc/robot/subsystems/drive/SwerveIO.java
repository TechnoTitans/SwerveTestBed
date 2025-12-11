package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.nio.ByteBuffer;

import static frc.robot.subsystems.drive.constants.SwerveConstants.ModuleCount;

public interface SwerveIO {
    class SwerveIOInputs implements LoggableInputs {
        public SwerveDriveState[] states = new SwerveDriveState[0];
        public Rotation3d gyroRotation3d = Rotation3d.kZero;

        @Override
        public void toLog(final LogTable table) {
            table.put("State", SwerveDriveState.struct, states);
            table.put("GyroRotation3d", Rotation3d.struct, gyroRotation3d);
        }

        @Override
        public void fromLog(final LogTable table) {
            final SwerveDriveState[] states = table.get("State", (SwerveDriveState[])null);
            this.states = states == null ? new SwerveDriveState[0] : states;
            this.gyroRotation3d = table.get("GyroRotation3d", Rotation3d.kZero);
        }
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see SwerveIOInputs
     * @see AutoLog
     */
    default void updateInputs(final SwerveIOInputs inputs) {}

    default void setControl(final SwerveRequest request) {}

    default void resetPose(final Pose2d pose) {}

    default void addVisionMeasurement(
            final Pose2d visionRobotPoseMeters,
            final double timestampSeconds,
            final Matrix<N3, N1> visionMeasurementStdDevs
    ) {}

    class SwerveDriveState extends SwerveDrivetrain.SwerveDriveState implements StructSerializable {
        public static final SwerveDriveStateStruct struct = new SwerveDriveStateStruct();
        public static final SwerveDriveState EmptyState = new SwerveDriveState();
        static {
            EmptyState.ModuleStates = new SwerveModuleState[ModuleCount];
            EmptyState.ModuleTargets = new SwerveModuleState[ModuleCount];
            EmptyState.ModulePositions = new SwerveModulePosition[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                EmptyState.ModuleStates[i] = new SwerveModuleState();
                EmptyState.ModuleTargets[i] = new SwerveModuleState();
                EmptyState.ModulePositions[i] = new SwerveModulePosition();
            }
        }

        public SwerveDriveState() {}

        public SwerveDriveState(final SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.Speeds = state.Speeds;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.RawHeading = state.RawHeading;
            this.Timestamp = state.Timestamp;
            this.OdometryPeriod = state.OdometryPeriod;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
        }
    }

    class SwerveDriveStateStruct implements Struct<SwerveDriveState> {
        @Override
        public Class<SwerveDriveState> getTypeClass() {
            return SwerveDriveState.class;
        }

        @Override
        public String getTypeName() {
            return "Swerve.SwerveDriveState";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() // Pose
                    + ChassisSpeeds.struct.getSize() // Speeds
                    + (ModuleCount * SwerveModuleState.struct.getSize()) // ModuleStates[4]
                    + (ModuleCount * SwerveModuleState.struct.getSize()) // ModuleTargets[4]
                    + (ModuleCount * SwerveModulePosition.struct.getSize()) // ModulePositions[4]
                    + Rotation2d.struct.getSize() // RawHeading
                    + kSizeDouble // Timestamp
                    + kSizeDouble // OdometryPeriod
                    + kSizeInt32 // SuccessfulDaqs
                    + kSizeInt32; // FailedDaqs
        }

        @Override
        public String getSchema() {
            return "Pose2d Pose; ChassisSpeeds Speeds; SwerveModuleState ModuleStates[4];"
                + "SwerveModuleState ModuleTargets[4]; SwerveModulePosition ModulePositions[4];"
                + "Rotation2d RawHeading; double Timestamp; double OdometryPeriod; int32 SuccessfulDaqs;"
                + "int32 FailedDaqs";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {
                    Pose2d.struct,
                    ChassisSpeeds.struct,
                    SwerveModuleState.struct,
                    SwerveModulePosition.struct,
                    Rotation2d.struct
            };
        }

        @Override
        public SwerveDriveState unpack(final ByteBuffer bb) {
            final SwerveDriveState state = new SwerveDriveState();
            state.Pose = Pose2d.struct.unpack(bb);
            state.Speeds = ChassisSpeeds.struct.unpack(bb);

            state.ModuleStates = new SwerveModuleState[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                state.ModuleStates[i] = SwerveModuleState.struct.unpack(bb);
            }

            state.ModuleTargets = new SwerveModuleState[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                state.ModuleTargets[i] = SwerveModuleState.struct.unpack(bb);
            }

            state.ModulePositions = new SwerveModulePosition[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                state.ModulePositions[i] = SwerveModulePosition.struct.unpack(bb);
            }

            state.RawHeading = Rotation2d.struct.unpack(bb);
            state.Timestamp = bb.getDouble();
            state.OdometryPeriod = bb.getDouble();
            state.SuccessfulDaqs = bb.getInt();
            state.FailedDaqs = bb.getInt();

            return state;
        }

        @Override
        public void pack(final ByteBuffer bb, final SwerveDriveState state) {
            Pose2d.struct.pack(bb, state.Pose);
            ChassisSpeeds.struct.pack(bb, state.Speeds);

            for (int i = 0; i < ModuleCount; i++) {
                SwerveModuleState.struct.pack(bb, state.ModuleStates[i]);
            }

            for (int i = 0; i < ModuleCount; i++) {
                SwerveModuleState.struct.pack(bb, state.ModuleTargets[i]);
            }

            for (int i = 0; i < ModuleCount; i++) {
                SwerveModulePosition.struct.pack(bb, state.ModulePositions[i]);
            }

            Rotation2d.struct.pack(bb, state.RawHeading);
            bb.putDouble(state.Timestamp);
            bb.putDouble(state.OdometryPeriod);
            bb.putInt(state.SuccessfulDaqs);
            bb.putInt(state.FailedDaqs);
        }
    }
}
