package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class TheRevMoter {
    /**
     *
     */
    PowerDistribution TheRevMoter = new PowerDistribution(1, ModuleType.kRev);

    public TheRevMoter(PowerDistribution theRevMoter) {
        TheRevMoter = theRevMoter;
    }

    public static double getTotalEnergy() {
        return 10;
    }

    public static double getTotalCurrent() {
        return 10;
    }

    public static double getTotalPower() {
        return 10;
    }
}
