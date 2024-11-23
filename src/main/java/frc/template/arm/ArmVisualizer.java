package frc.template.arm;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Degrees;

public class ArmVisualizer {
    public final Mechanism2d mechanism;
    public final MechanismLigament2d ligament;

    public ArmVisualizer(Color color, double width, double height, double length) {
        mechanism = new Mechanism2d(width, height, new Color8Bit(Color.kGray));
        var root = mechanism.getRoot("Root", width / 2, height / 2);
        ligament = root.append(new MechanismLigament2d("Pivot", length, 0.0, 4.0, new Color8Bit(color)));
    }

    public void update(Measure<Angle> armPosition) {
        ligament.setAngle(armPosition.in(Degrees));
    }
}
