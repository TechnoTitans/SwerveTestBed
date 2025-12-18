package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.commands.RobotModeLoggedTriggers;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.logging.LoggedCommandScheduler;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class Robot extends LoggedRobot {
    protected static final String LogKey = "Robot";
    private static final String AKitLogPath = "/U/logs";
    private static final String HootLogPath = "/U/logs";

    public static final BooleanSupplier IsRedAlliance = () -> {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    };

    public final PowerDistribution powerDistribution = new PowerDistribution(
            RobotMap.PowerDistributionHub, PowerDistribution.ModuleType.kRev
    );

    public final Swerve swerve = new Swerve(
            Constants.CURRENT_MODE,
            SwerveConstants.CTRESwerve.DrivetrainConstants,
            new SwerveConstants.SwerveModuleConfig[]{
                    SwerveConstants.FrontLeftModule,
                    SwerveConstants.FrontRightModule,
                    SwerveConstants.BackLeftModule,
                    SwerveConstants.BackRightModule
            },
            SwerveConstants.CTRESwerve.FrontLeft,
            SwerveConstants.CTRESwerve.FrontRight,
            SwerveConstants.CTRESwerve.BackLeft,
            SwerveConstants.CTRESwerve.BackRight
    );

    public final PhotonVision photonVision = new PhotonVision(
            Constants.RobotMode.DISABLED,
            swerve
    );

    public final Autos autos = new Autos(
            swerve,
            photonVision
    );
    public final AutoChooser autoChooser = new AutoChooser(
            new AutoOption(
                    "DoNothing",
                    autos::doNothing,
                    Constants.CompetitionType.COMPETITION
            )
    );

    public final CommandXboxController driverController = new CommandXboxController(RobotMap.MainController);
    public final CommandXboxController coController = new CommandXboxController(RobotMap.CoController);
    public final Alert driverControllerDisconnected = new Alert(
            "Driver controller not connected!",
            Alert.AlertType.kWarning
    );
    public final Alert coControllerDisconnected = new Alert(
            "Co controller not connected!",
            Alert.AlertType.kWarning
    );

    private final EventLoop teleopEventLoop = new EventLoop();
    private final EventLoop testEventLoop = new EventLoop();

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
    private final LoggedTrigger disabled = RobotModeLoggedTriggers.disabled(group);
    private final LoggedTrigger teleopEnabled = RobotModeLoggedTriggers.teleop(group);
    private final LoggedTrigger autonomousEnabled = RobotModeLoggedTriggers.autonomous(group);
    private final LoggedTrigger endgameTrigger = group.t("endgame", () -> DriverStation.getMatchTime() <= 20)
            .and(DriverStation::isFMSAttached)
            .and(RobotModeTriggers.teleop());

    @Override
    public void robotInit() {
        if ((RobotBase.isReal() && Constants.CURRENT_MODE != Constants.RobotMode.REAL) ||
                (RobotBase.isSimulation() && Constants.CURRENT_MODE == Constants.RobotMode.REAL)) {
            DriverStation.reportWarning(
                    String.format(
                            "Potentially incorrect CURRENT_MODE \"%s\" specified, robot is running \"%s\"",
                            Constants.CURRENT_MODE,
                            RobotBase.getRuntimeType().toString()
                    ),
                    true
            );

            throw new RuntimeException("Incorrect CURRENT_MODE specified!");
        }

        // we never use LiveWindow, and apparently this causes loop overruns so disable it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // register shutdown hook
        ToClose.hook();

        // disable joystick not found warnings when in sim
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE == Constants.RobotMode.SIM);

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                try {
                    Files.createDirectories(Paths.get(HootLogPath));
                    SignalLogger.setPath(HootLogPath);
                } catch (final IOException ioException) {
                    SignalLogger.setPath("/U");
                    DriverStation.reportError(
                            String.format(
                                    "Failed to create .hoot log path at \"%s\"! Falling back to default.\n%s",
                                    HootLogPath,
                                    ioException
                            ),
                            false
                    );
                }

                Logger.addDataReceiver(new WPILOGWriter(AKitLogPath));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // log to working directory when running sim
                // setPath doesn't seem to work in sim (path is ignored and hoot files are always sent to /logs)
//                SignalLogger.setPath("/logs");
                Logger.addDataReceiver(new WPILOGWriter(""));
                Logger.addDataReceiver(new NT4Publisher());

                DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
                DriverStationSim.notifyNewData();

                autonomousEnabled.whileTrue(
                        Commands.waitSeconds(15)
                                .andThen(() -> {
                                    DriverStationSim.setEnabled(false);
                                    DriverStationSim.notifyNewData();
                                })
                );
            }
            case REPLAY -> {
                setUseTiming(false);

                final String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(
                                LogFileUtil.addPathSuffix(logPath, "_sim"),
                                WPILOGWriter.AdvantageScopeOpenBehavior.AUTO)
                );
            }
        }

        powerDistribution.clearStickyFaults();
        powerDistribution.setSwitchableChannel(true);

        configureStateTriggers();
        configureAutos();
        configureButtonBindings(teleopEventLoop);

        LoggedCommandScheduler.init(CommandScheduler.getInstance());

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();
        ToClose.add(SignalLogger::stop);

        Logger.start();

        Logger.recordOutput("EmptyPose", Pose3d.kZero);
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        RefreshAll.refreshAll();

        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();

        driverControllerDisconnected.set(!driverController.getHID().isConnected());
        coControllerDisconnected.set(!coController.getHID().isConnected());

        LoggedCommandScheduler.periodic();

        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        //noinspection SuspiciousNameCombination
        swerve.setDefaultCommand(
                swerve.teleopDriveCommand(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX
                )
        );
    }

    @Override
    public void teleopPeriodic() {
        teleopEventLoop.poll();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        driverController.leftBumper(testEventLoop).onTrue(Commands.runOnce(SignalLogger::stop));
    }

    @Override
    public void testPeriodic() {
        testEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {}

    public void configureStateTriggers() {
        endgameTrigger.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1)
        );

        disabled.onTrue(swerve.stopCommand());
    }

    public void configureAutos() {
        autonomousEnabled.whileTrue(Commands.deferredProxy(() -> autoChooser.getSelected().cmd()));

        autoChooser.addAutoOption(new AutoOption(
                "Straight",
                autos::straight
        ));
    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        this.driverController.rightBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.FAST),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ).withName("SwerveSpeedFast"));

        this.driverController.leftBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SLOW),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ).withName("SwerveSpeedSlow"));

        this.driverController.y().onTrue(Commands.runOnce(() -> photonVision.resetPose(Pose2d.kZero)));
    }
}
