package frc.robot.subsystems.intakeRoller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.PIDF;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.intakeRollerConfig;

public class IntakeRollerIOSim extends IntakeRollerIO {

    private final DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1)
                    , 0.01, intakeRollerConfig.gearRatio())
            , DCMotor.getKrakenX60(1), 0.004, 0.0
    );
    private OperatorDashboard operatorDashboard = OperatorDashboard.get();


    private RobotState robotState = RobotState.get();
    private PIDController positionPid = intakeRollerConfig.positionGains().toPID();
    public static IntakeSimulation intakeSimulation =
            IntakeSimulation.OverTheBumperIntake(
                    // Specify the type of game pieces that the intake can collect
                    "Coral",
                    // Specify the drivetrain to which this intake is attached
                    ModuleIOSim.driveSimulation,
                    // Width of the intake
                    Meters.of(driveConfig.trackWidthMeters()),
                    // The extension length of the intake beyond the robot's frame (when activated)
                    Inches.of(10),
                    // The intake is mounted on the back side of the chassis
                    IntakeSimulation.IntakeSide.FRONT,
                    // The intake can hold up to 1 note
                    1);
    SimulatedArena arena = SimulatedArena.getInstance();


    public IntakeRollerIOSim() {
    }


    private PIDController velocityPid = intakeRollerConfig.velocityGains().toPID();
    private SimpleMotorFeedforward velocityFeedforward = intakeRollerConfig.velocityGains().toSimpleFF();
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private final Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier
            =
            operatorDashboard::getSelectedCoralScoringLevel;

    private double appliedVolts;
    private boolean closedLoop = true;
    private boolean positionControl = false;
    private double ffVolts;


    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {

        if (closedLoop) {
            if (positionControl) {
                appliedVolts = positionPid.calculate(
                        motorSim.getAngularPositionRad());
            } else {
                appliedVolts = velocityPid.calculate(
                        motorSim.getAngularVelocityRadPerSec()) + ffVolts;
            }
        }
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

        motorSim.setInputVoltage(appliedVolts);

        motorSim.update(0.02);

        inputs.connected = true;
        inputs.isCoralIn = intakeSimulation.getGamePiecesAmount() != 0;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

        if (IntakeRoller.get().getIntakeRollerGoal() == IntakeRoller.IntakeRollerGoal.INTAKE) {

            intakeSimulation.startIntake();

            //  intakeSimulation.stopIntake();


        } else {
            intakeSimulation.stopIntake();

        }

        // System.out.println(intakeSimulation.getGamePiecesAmount());

////

//        var arena = org.ironmaple.simulation.SimulatedArena.getInstance();
//
//        if (arena != null) {
//
//            Translation3d rollerTip3d = robotMechanism.intakeRoller.getRollerTip3d();
//            Translation2d rollerTip2d = robotMechanism.intakeRoller.getRollerTip2d();

//        GamePieceOnFieldSimulation[] corals = arena.gamePiecesOnField().stream()
//                .filter(gp -> gp.type.equals("Coral"))
//                .toArray(GamePieceOnFieldSimulation[]::new);
//            GamePieceOnFieldSimulation[] allGamePieces = arena.gamePiecesOnField().toArray(new GamePieceOnFieldSimulation[0]);
//
//        for (GamePieceOnFieldSimulation piece : allGamePieces) {
//            for (GamePieceOnFieldSimulation piece : arena.gamePiecesOnField()) {
//
//                if (!piece.type.equals("Coral")) continue;
//
//
//                Translation2d coralPos2d = piece.getPose3d().toPose2d().getTranslation();
//                Translation3d coralPos3d = piece.getPose3d().getTranslation();
//                Pose3d coral3Pos = piece.getPose3d();
//                Pose2d coral2Pos = piece.getPose3d().toPose2d();
//
//                if (distance <= robotMechanism.intakeRoller.rollerLength
//
//                var distance = coralPos3d.getDistance(rollerTip3d);
//                if (piece.getPose3d().getTranslation().getDistance(rollerTip3d) <= robotMechanism.intakeRoller.rollerLength
//                ) {
//                    arena.removeGamePiece(piece);

//
//            arena.addGamePieceProjectile(
//                    new ReefscapeCoralOnFly(
//                            ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
//                            rollerTip2d,
//                            ModuleIOSim.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
//                            ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose().getRotation()
//                            ,
//
//                            Meters.of(0.05),
//                            MetersPerSecond.of(0),
//                            Degrees.of(0)
//                    )
//            );


    }


    @Override
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting IntakeRoller position gains");
        positionPid = newGains.toPID();
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting IntakeRoller velocity gains");
        velocityFeedforward = newGains.toSimpleFF();
        velocityPid = newGains.toPID();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting IntakeRoller brake mode to " + enable);
    }

    @Override
    public void setOpenLoop(double output) {
        appliedVolts = output;
        closedLoop = false;
    }

//    @Override // Defined by IntakeIO
//    public void setRunning(boolean runIntake) {
//
//    }

    @Override
    public void setPosition(double positionRad) {
        closedLoop = true;
        positionControl = true;
        positionPid.setSetpoint(positionRad);
    }

    //
    @Override
    public void spawnCoral() {
//        Translation2d spawnPos = robotState.getPose().transformBy(
//                new Transform2d(-1.5, 0.0, new Rotation2d())
//        ).getTranslation();
//        new Translation2d(robotState.getPose()
//                .getTranslation().getX() - 1.5
//                , 0.0)


//        Translation2d spawnPos = robotState.getPose().
//                getTranslation().plus(new Translation2d(-1.5, 0.0).rotateBy(robotState.getRotation()));

        if (intakeSimulation.getGamePiecesAmount() != 0) {
            arena.addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                            robotState.getPose().getTranslation(),
                            new Translation2d(robotState.getPose()
                                    .getTranslation().getX() - Units.inchesToMeters(2)
                                    , 0.0)
                            ,
                            ModuleIOSim.driveSimulation.
                                    getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            robotState.getPose().getRotation()
                            ,

                            Meters.of(coralScoringLevelSupplier.get().coralScoringGoal.getAsDouble()),
                            MetersPerSecond.of(-1.0),
                            coralScoringLevelSupplier.get() ==
                                    OperatorDashboard.CoralScoringLevel.L1
                                    ? Degrees.of(0) :
                                    Degrees.of(65)));
            intakeSimulation.obtainGamePieceFromIntake();
        }
    }


    @Override
    public void setVelocity(double velocityRadPerSec) {
        closedLoop = true;
        positionControl = false;
        ffVolts = velocityFeedforward.calculate(velocityRadPerSec);
        velocityPid.setSetpoint(velocityRadPerSec);
    }


}
