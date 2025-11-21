package frc.robot.subsystems.drive.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotMechanism;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.lib.commands.CommandsExt.runOnceAndWaitUntil;
import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.drive.intake.CoralIntakeConstants.pivotLengthMeters;
import static frc.robot.subsystems.drive.intake.CoralIntakeConstants.pivotSetpointToleranceRad;


public class CoralIntake implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private static final PivotIO pivotIO = CoralIntakeConstants.pivotIo;
    private static final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    public enum PivotGoal {
        CHARACTERIZATION(null),
        STOW(() -> 1.353),
        INTAKE(() -> 0.12833586);

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRad;

        PivotGoal(DoubleSupplier setpointRad) {
            this.setpointRad = setpointRad;
        }
    }


    private PivotGoal pivotGoal = PivotGoal.STOW;
    private static CoralIntake instance;

    public static CoralIntake get() {
        if (instance == null)
            synchronized (CoralIntake.class) {
                instance = new CoralIntake();
            }

        return instance;
    }

    /* private final RollersIO rollersIO = CoralIntakeConstants.rollersIo;
     private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

     @RequiredArgsConstructor
     public enum RollersGoal {
         CHARACTERIZATION(null),
         IDLE(() -> 0),
         INTAKE(() -> Units.rotationsPerMinuteToRadiansPerSecond(-200)),
         EJECT(() -> 1);

         private final DoubleSupplier setpointRadPerSec;
     }

     @Getter
     private RollersGoal rollersGoal = RollersGoal.IDLE;

     private static CoralIntake instance;

     public static CoralIntake get() {
         if (instance == null)
             synchronized (CoralIntake.class) {
                 instance = new CoralIntake();
             }

         return instance;
     }
 */

    private CoralIntake() {
    }

    @Override
    public void periodicBeforeCommands() {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Inputs/CoralIntake/Pivot", pivotInputs);

       /* rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/CoralIntake/Rollers", rollersInputs);
*/
        RobotMechanism.CoralIntake.root.setPosition(
                middleOfRobot + Units.inchesToMeters(6.5) + pivotLengthMeters * Math.cos(pivotInputs.positionRad),
                Units.inchesToMeters(5.5) + pivotLengthMeters * Math.sin(pivotInputs.positionRad)
        );
        //RobotMechanism.CoralIntake.topRollersLigament.setAngle(Units.radiansToDegrees(rollersInputs.positionRad));
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("CoralIntake/Pivot/Goal", pivotGoal);
        if (pivotGoal.setpointRad != null) {
            var pivotSetpointRad = pivotGoal.setpointRad.getAsDouble();
            pivotIO.setClosedLoop(pivotSetpointRad);
            Logger.recordOutput("CoralIntake/Pivot/ClosedLoop", true);
            Logger.recordOutput("CoralIntake/Pivot/SetpointRad", pivotSetpointRad);
        } else {
            Logger.recordOutput("CoralIntake/Pivot/ClosedLoop", false);
        }
/*
        ////////////// ROLLERS //////////////
        Logger.recordOutput("CoralIntake/Rollers/Goal", rollersGoal);
        // TODO: disabled check
        if (rollersGoal.setpointRadPerSec != null) {
            var rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setVelocity(rollersSetpointRadPerSec);
            Logger.recordOutput("CoralIntake/Rollers/ClosedLoop", true);
            Logger.recordOutput("CoralIntake/Rollers/SetpointRadPerSec", rollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("CoralIntake/Rollers/ClosedLoop", false);
        }*/
    }

    public Command setGoals(PivotGoal pivotGoal/*, RollersGoal rollersGoal*/) {
        return runOnce(() -> {
            this.pivotGoal = pivotGoal;
            /*this.rollersGoal = rollersGoal;*/
        });
    }

    private boolean atPivotGoal() {
        return pivotGoal.setpointRad != null && Math.abs(pivotGoal.setpointRad.getAsDouble() - pivotInputs.positionRad) <= pivotSetpointToleranceRad;
    }

    public Command waitUntilAtPivotGoal() {
        return waitUntil(this::atPivotGoal);
    }

    public Command setGoalsAndWaitUntilAtPivotGoal(PivotGoal pivotGoal/*, RollersGoal rollersGoal*/) {
        return runOnceAndWaitUntil(
                () -> {
                    this.pivotGoal = pivotGoal;
                    //this.rollersGoal = rollersGoal;
                },
                this::atPivotGoal
        );
    }
}
