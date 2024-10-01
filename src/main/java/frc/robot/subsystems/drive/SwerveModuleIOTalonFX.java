package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.utils.ctre.Phoenix6Utils;

import static frc.robot.subsystems.drive.constants.SwerveConstants.Config;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final double driveReduction = Config.driveReduction();
    private final double turnReduction = Config.turnReduction();
    private final double couplingRatio = Config.couplingRatio();

    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final PositionVoltage positionVoltage;

    private final OdometryThreadRunner odometryThreadRunner;
    // Cached StatusSignals
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveTorqueCurrent;
    private final StatusSignal<Double> driveDeviceTemp;
    private final StatusSignal<Double> turnPosition;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnTorqueCurrent;
    private final StatusSignal<Double> turnDeviceTemp;

    // Odometry StatusSignal update buffers
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer drivePositionSignalBuffer;
    private final DoubleCircularBuffer turnPositionSignalBuffer;

    public SwerveModuleIOTalonFX(
            final SwerveConstants.SwerveModuleConstants constants,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.driveMotor = new TalonFX(constants.driveMotorId(), constants.moduleCANBus());
        this.turnMotor = new TalonFX(constants.turnMotorId(), constants.moduleCANBus());

        this.turnEncoder = new CANcoder(constants.turnEncoderId(), constants.moduleCANBus());
        this.magnetOffset = constants.turnEncoderOffsetRots();

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.odometryThreadRunner = odometryThreadRunner;
        this.odometryThreadRunner.registerControlRequest(driveMotor, velocityTorqueCurrentFOC, driveMotor::setControl);
        this.odometryThreadRunner.registerControlRequest(turnMotor, positionVoltage, turnMotor::setControl);

        this.drivePosition = driveMotor.getPosition();
        this.driveVelocity = driveMotor.getVelocity();
        this.driveTorqueCurrent = driveMotor.getTorqueCurrent();
        this.driveDeviceTemp = driveMotor.getDeviceTemp();
        this.turnPosition = turnMotor.getPosition();
        this.turnVelocity = turnMotor.getVelocity();
        this.turnTorqueCurrent = turnMotor.getTorqueCurrent();
        this.turnDeviceTemp = turnMotor.getDeviceTemp();

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.drivePositionSignalBuffer = odometryThreadRunner.registerSignal(driveMotor, this.drivePosition);
        this.turnPositionSignalBuffer = odometryThreadRunner.registerSignal(turnMotor, this.turnPosition);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        // TODO: check StatusCode of some/most of these blocking config calls... maybe retry if failed?
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        // TODO: drive and azimuth gains both need to be re-tuned
        driveTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKS(5.6753)
                .withKV(0)
                .withKA(2.5488)
                .withKP(46.391);
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = driveReduction;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        turnTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKP(45)
                .withKS(0.5);
        turnTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        turnTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        turnTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        turnTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnTalonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = turnReduction;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);

        velocityTorqueCurrentFOC.UpdateFreqHz = 0;
        positionVoltage.UpdateFreqHz = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                turnEncoder.getPosition(),
                turnEncoder.getVelocity()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                this.driveVelocity,
                this.driveTorqueCurrent,
                this.driveDeviceTemp,
                this.turnVelocity,
                this.turnTorqueCurrent,
                this.turnDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor, turnEncoder);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                this.drivePosition,
                this.driveVelocity,
                this.driveTorqueCurrent,
                this.driveDeviceTemp,
                this.turnPosition,
                this.turnVelocity,
                this.turnTorqueCurrent,
                this.turnDeviceTemp
        );

        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = this.driveVelocity.getValue();
        inputs.driveTorqueCurrentAmps = this.driveTorqueCurrent.getValue();
        inputs.driveTempCelsius = this.driveDeviceTemp.getValue();

        inputs.turnPositionRots = getRawAngle();
        inputs.turnVelocityRotsPerSec = this.turnVelocity.getValue();
        inputs.turnTorqueCurrentAmps = this.turnTorqueCurrent.getValue();
        inputs.turnTempCelsius = this.turnDeviceTemp.getValue();

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryDrivePositionsRots = OdometryThreadRunner.writeBufferToArray(drivePositionSignalBuffer);
        drivePositionSignalBuffer.clear();

        inputs.odometryTurnPositionRots = OdometryThreadRunner.writeBufferToArray(turnPositionSignalBuffer);
        turnPositionSignalBuffer.clear();
    }

    /**
     * Get the measured wheel angle (mechanism) angle of the {@link SwerveModuleIO}, in raw units (rotations)
     *
     * @return the measured wheel (turner) angle, in rotations
     */
    private double getRawAngle() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(this.turnPosition, this.turnVelocity);
    }

    /**
     * Get the measured drive wheel (mechanism) position of the {@link SwerveModuleIO}, in raw units (rotations)
     *
     * @return the measured drive wheel position, in rotations
     */
    public double getDrivePosition() {
        final double driveWheelPosition = Phoenix6Utils.latencyCompensateIfSignalIsGood(
                this.drivePosition, this.driveVelocity
        );
        final double turnPosition = Phoenix6Utils.latencyCompensateIfSignalIsGood(
                this.turnPosition, this.turnVelocity
        );
        final double driveBackOutWheelRotations = (
                (turnPosition * couplingRatio)
                        / driveReduction
        );

        return driveWheelPosition - driveBackOutWheelRotations;
    }

    @Override
    public void setInputs(final double desiredDriverVelocity, final double desiredTurnerRotations) {
        final double driveVelocityBackOut = (
                (this.turnVelocity.getValue() * couplingRatio)
                        / driveReduction
        );
        final double backedOutDriveVelocity = desiredDriverVelocity + driveVelocityBackOut;

        odometryThreadRunner.updateControlRequest(driveMotor, velocityTorqueCurrentFOC);
        driveMotor.setControl(velocityTorqueCurrentFOC
                .withVelocity(backedOutDriveVelocity)
        );
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setDriveCharacterizationVolts(double driveVolts, double desiredTurnerRotations) {
        odometryThreadRunner.updateControlRequest(driveMotor, voltageOut);
        driveMotor.setControl(voltageOut.withOutput(driveVolts));
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setDriveCharacterizationAmps(double driveTorqueCurrentAmps, double desiredTurnerRotations) {
        odometryThreadRunner.updateControlRequest(driveMotor, torqueCurrentFOC);
        driveMotor.setControl(torqueCurrentFOC.withOutput(driveTorqueCurrentAmps));
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        final StatusCode refreshCode = driveMotor.getConfigurator().refresh(turnTalonFXConfiguration);
        if (!refreshCode.isOK()) {
            DriverStation.reportWarning(
                    String.format(
                            "Failed to set NeutralMode on TalonFX %s (%s)",
                            driveMotor.getDeviceID(),
                            driveMotor.getNetwork()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
