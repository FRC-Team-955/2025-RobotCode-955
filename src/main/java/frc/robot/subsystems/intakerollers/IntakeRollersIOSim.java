package frc.robot.subsystems.intakerollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.PIDF;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.IntakeSimulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.intakeRollersConfig;

public class IntakeRollersIOSim extends IntakeRollersIO {

    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.01,
                intakeRollersConfig.gearRatio()
        ),
            DCMotor.getKrakenX60(1),
            0.004,
            0.0
    );

//    private PIDController positionPID = intakeRollersConfig.positionGains().toPID();
    private PIDController velocityPID = intakeRollersConfig.velocityGains().toPID();
    private SimpleMotorFeedforward velocityFeedforward = intakeRollersConfig.velocityGains().toSimpleFF();

    private boolean closedLoop = true;
    private double appliedVolts;
//    private boolean positionControl = false;
    private double ffVolts;

    public IntakeRollersIOSim() {}

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        if (closedLoop) {
//            if (positionControl) {
//                appliedVolts = positionPID.calculate(motorSim.getAngularPositionRad());
//            } else {
                appliedVolts = velocityPID.calculate(motorSim.getAngularAccelerationRadPerSecSq()) + ffVolts;
//            }
        }

        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

//        if (IntakeRollers.get().getCurrentGoal() == IntakeRollers.IntakeRollersGoal.INTAKE) {
//            intakeSimulation.startIntake();
//        } else {
//            intakeSimulation.stopIntake();
//        }
//
//        var intakedCoral = intakeSimulation.getGamePiecesAmount() > 0;
////        if (intakedCoral) {
//            intakeSimulation.obtainGamePieceFromIntake();
//        }
    }

//    @Override
//    public void setPositionPIDF(PIDF newGains) {
//        System.out.println("Setting roller position gains");
//        positionPID = newGains.toPID();
//    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting roller velocity gains");
        velocityFeedforward = newGains.toSimpleFF();
        velocityPID = newGains.toPID();
    }

    @Override
    public void setOpenLoop(double output) {
        closedLoop = false;
        appliedVolts = output;
    }

//    @Override
//    public void setPosition(double positionRad) {
//        closedLoop = true;
//        positionControl = true;
//        positionPID.setSetpoint(positionRad);
//    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        closedLoop = true;
//        positionControl = false;
        ffVolts = velocityFeedforward.calculate(velocityRadPerSec);
        velocityPID.setSetpoint(velocityRadPerSec);
    }
}
