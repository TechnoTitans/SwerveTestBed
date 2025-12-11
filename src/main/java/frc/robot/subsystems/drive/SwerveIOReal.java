package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import frc.robot.constants.Constants;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveIOReal implements SwerveIO {
    private final Lock stateLock;
    private final CircularBuffer<SwerveDrivetrain.SwerveDriveState> stateBuffer;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    public SwerveIOReal(
            final SwerveDrivetrainConstants drivetrainConstants,
            final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.stateLock = new ReentrantLock();
        this.stateBuffer = new CircularBuffer<>(20);
        this.drivetrain = new SwerveDrivetrain<>(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, 250,
                Constants.Vision.STATE_STD_DEVS,
                VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(80)),
                moduleConstants
        );
        this.drivetrain.registerTelemetry(state -> {
            try {
                stateLock.lock();
                stateBuffer.addFirst(state.clone());
            } finally {
                stateLock.unlock();
            }
        });
    }

    @Override
    public void updateInputs(final SwerveIOInputs inputs) {
        try {
            stateLock.lock();

            final int nStates = stateBuffer.size();
            final SwerveDriveState[] states = new SwerveDriveState[nStates];
            for (int i = 0; i < nStates; i++) {
                states[i] = new SwerveDriveState(stateBuffer.get(i));
            }

            stateBuffer.clear();
            inputs.states = states;
        } finally {
            stateLock.unlock();
        }

        inputs.gyroRotation3d = drivetrain.getRotation3d();
        inputs.currentTimeSecondsCTRE = Utils.getCurrentTimeSeconds();
    }

    @Override
    public void setControl(final SwerveRequest request) {
        drivetrain.setControl(request);
    }

    @Override
    public void resetPose(final Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(
            final Pose2d visionRobotPoseMeters,
            final double timestampSecondsCTRE,
            final Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        drivetrain.addVisionMeasurement(
                visionRobotPoseMeters,
                timestampSecondsCTRE,
                visionMeasurementStdDevs
        );
    }

    @Override
    public void setOperatorPerspectiveForward(final Rotation2d forwardDirection) {
        drivetrain.setOperatorPerspectiveForward(forwardDirection);
    }
}
