package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.climber.ClimberConstants.gains;


public class ClimberTuning {
    public static final PIDF.Tunable gainsTunable = gains.tunable("Climber/Gains");

    public static final LoggedTunableNumber stowGoalSetpoint =
            new LoggedTunableNumber("Climber/Goal/Stow", Units.degreesToRadians(90));
}
