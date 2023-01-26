package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class TheRevMoter {
    /**
     *
     */
    private static PowerDistribution distroPannel = new PowerDistribution(2, ModuleType.kCTRE);

    private static WPI_TalonFX rightMotor = new WPI_TalonFX(16);

    private TheRevMoter() {
    }

    public static void driveTheMotor() { rightMotor.set(0.2); }

    public static PowerDistribution getInstance() { return distroPannel; }

    public static double getTotalEnergy() {
        return TheRevMoter.getInstance().getVoltage();
    }

    public static double getTotalCurrent() {
        return TheRevMoter.getInstance().getTotalCurrent();
    }

    public static double getTotalPower() {
        return TheRevMoter.getInstance().getTotalPower();
    }
}
