package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.DoubleCircularBuffer;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.utils.control.DeltaTime;

public class GyroIOSim implements GyroIO {
    public static final double USE_SIMULATED_PITCH = 0;
    public static final double USE_SIMULATED_ROLL = 0;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSimState;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] swerveModules;
    private final double[] lastSwerveModulePositionMeters = {0.0, 0.0, 0.0, 0.0};

    private final DeltaTime deltaTime;
    private final LinearFilter filter = LinearFilter.movingAverage(50);
    private Rotation2d rawGyroYaw = Rotation2d.fromDegrees(0);

    // Cached StatusSignals
    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> pitch;
    private final StatusSignal<Double> roll;
    private final StatusSignal<Double> yawVelocity;
    private final StatusSignal<Double> pitchVelocity;
    private final StatusSignal<Double> rollVelocity;
    private final StatusSignal<Boolean> faultHardware;

    // StatusSignal buffers for high-freq odometry
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer yawSignalBuffer;

    public GyroIOSim(
            final HardwareConstants.GyroConstants gyroConstants,
            final OdometryThreadRunner odometryThreadRunner,
            final SwerveDriveKinematics kinematics,
            final SwerveModule[] swerveModules
    ) {
        this.pigeon = new Pigeon2(gyroConstants.gyroId(), gyroConstants.CANBus());
        this.pigeonSimState = pigeon.getSimState();
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;

        this.deltaTime = new DeltaTime(true);

        this.yaw = pigeon.getYaw();
        this.pitch = pigeon.getPitch();
        this.roll = pigeon.getRoll();
        this.yawVelocity = pigeon.getAngularVelocityZWorld();
        this.pitchVelocity = pigeon.getAngularVelocityXWorld();
        this.rollVelocity = pigeon.getAngularVelocityYWorld();
        this.faultHardware = pigeon.getFault_Hardware();

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.yawSignalBuffer = odometryThreadRunner.registerSignal(pigeon, yaw);

        pigeonSimState.setSupplyVoltage(12);
        pigeonSimState.setPitch(USE_SIMULATED_PITCH);
        pigeonSimState.setRoll(USE_SIMULATED_ROLL);
    }

    private void updateGyro(final double dtSeconds) {
        final SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = swerveModules[i].getState();
        }

        rawGyroYaw = rawGyroYaw.plus(Rotation2d.fromRadians(
                kinematics.toChassisSpeeds(moduleStates).omegaRadiansPerSecond * dtSeconds
        ));
        pigeonSimState.setRawYaw(rawGyroYaw.getDegrees());
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        //TODO fill in correct mount pose
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseRoll = 0;
        pigeon2Configuration.MountPose.MountPosePitch = 0;
        pigeon2Configuration.MountPose.MountPoseYaw = 0;
        pigeon.getConfigurator().apply(pigeon2Configuration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pitch,
                pitchVelocity,
                roll,
                rollVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                faultHardware
        );
        ParentDevice.optimizeBusUtilizationForAll(pigeon);
    }

    @Override
    public void periodic() {
        updateGyro(deltaTime.get());
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                yaw,
                pitch,
                roll,
                yawVelocity,
                pitchVelocity,
                rollVelocity,
                faultHardware
        );

        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();
        inputs.yawVelocityDegPerSec = yawVelocity.getValue();
        inputs.pitchVelocityDegPerSec = pitchVelocity.getValue();
        inputs.rollVelocityDegPerSec = rollVelocity.getValue();
        inputs.hasHardwareFault = faultHardware.getValue();

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryYawPositionsDeg = OdometryThreadRunner.writeBufferToArray(yawSignalBuffer);
        yawSignalBuffer.clear();
    }

    public double getYaw() {
        // TODO: fairly sure that yaw, pitch, roll velocity don't work in sim, so we can't latency compensate
//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                yaw,
//                getYawVelocitySignal()
//        );
        return yaw.getValue();
    }

    public double getPitch() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getPitch(),
//                getPitchVelocitySignal()
//        );
        return pitch.getValue();
    }

    public double getRoll() {
        // see previous mention of velocities not working in sim

//        return Phoenix6Utils.latencyCompensateIfSignalIsGood(
//                pigeon.getRoll(),
//                getRollVelocitySignal()
//        );
        return roll.getValue();
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
