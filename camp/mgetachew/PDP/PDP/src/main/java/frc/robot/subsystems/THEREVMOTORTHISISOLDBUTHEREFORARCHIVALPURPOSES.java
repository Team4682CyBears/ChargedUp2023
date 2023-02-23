/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Timer;
import java.util.TimerTask;

public class TheRevMoter extends SubsystemBase {

    private final double m_increment = 0.1;
    private static WPI_TalonFX rightMotor = new WPI_TalonFX(0);
    private static double motormovespeedby = 0.0;
    private Timer timer;
    private ShuffleboardTab tab = Shuffleboard.getTab("Right Motor");

    public TheRevMoter() {
        timer = new Timer();
        timer.schedule(new DriveMotorTask(), 0, 5000); // schedule the task to run every 10 seconds
    }

    private class DriveMotorTask extends TimerTask {
        @Override
        public void run() {
            driveTheMotor();
        }
    }

    public void driveTheMotor() {
        motormovespeedby += m_increment;
        rightMotor.set(motormovespeedby);
    }

    public void setMotorSpeedFromDashboard() {
    }

    public void stopMotor() {
        rightMotor.stopMotor();
    }

    // from https://www.chiefdelphi.com/t/smartdashboard-input-output/162502/4 100% not plagiarism
    private double createSmartDashboardNumber(String key, double defValue) {

        // See if already on dashboard, and if so, fetch current value
        double value = SmartDashboard.getNumber(key, defValue);

        // Make sure value is on dashboard, puts back current value if already set
        // otherwise puts back default value
        SmartDashboard.putNumber(key, value);

        return value;
    }

}

*/