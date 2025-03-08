package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMechanism;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.ArrayList;

import static frc.robot.subsystems.leds.LEDConstants.length;

public class LEDsIOSim extends LEDsIO {
    private final ArrayList<LoggedMechanismLigament2d> ledLigaments = new ArrayList<>();

    public LEDsIOSim() {
        RobotMechanism robotMechanism = RobotMechanism.get();
        // to whoever is trying to debug this in the future: good luck!
        double rightBottomY = 1.1;
        double leftTopY = 1 + length / 2.0 * 0.1;
        for (int index = 0; index < length; index++) {
            boolean onRightSide = index > length / 2 - 1;
            String name = "LED" + index;
            int localIndex = index % (length / 2);
            LoggedMechanismRoot2d root = robotMechanism.mechanism.getRoot(
                    name,
                    RobotMechanism.middleOfRobot + (onRightSide ? 0.1 : -0.1),
                    onRightSide
                            ? rightBottomY + localIndex * 0.1
                            : leftTopY - localIndex * 0.1
            );
            ledLigaments.add(root.append(new LoggedMechanismLigament2d(
                    name,
                    0.1,
                    90,
                    10,
                    new Color8Bit(Color.kBlack)
            )));
        }
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            // https://github.com/FRC-Team-955/2024-RobotCode-749/blob/kotlin-old/src/main/java/frc/robot/subsystems/leds/LEDs.kt#L88
//            System.out.print("\u001b[38;2;" + buffer.getRed(i) + ";" + buffer.getGreen(i) + ";" + buffer.getBlue(i) + "m■\u001b[0m");
            ledLigaments.get(i).setColor(new Color8Bit(
                    buffer.getRed(i),
                    buffer.getGreen(i),
                    buffer.getBlue(i)
            ));
        }
//        System.out.println();
    }
}