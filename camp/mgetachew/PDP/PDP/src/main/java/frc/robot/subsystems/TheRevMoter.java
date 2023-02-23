package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Timer;
import java.util.TimerTask;

public class TheRevMoter extends SubsystemBase {
    private final double m_increment = 0.0005;
    private static WPI_TalonFX rightMotor = new WPI_TalonFX(0);
    private static double motormovespeedby = 0.0;
    private Timer timer;

    // TBC this wont work atm =<
    public TheRevMoter() {
        timer = new Timer();
        timer.schedule(new MotorTask(), 0, 10000);
    }
// Same here
    private class MotorTask extends TimerTask {
        public void run() {
            driveTheMotor();
        }
    }

    public void driveTheMotor() { 
        motormovespeedby += m_increment;
      if (motormovespeedby > 1.0) {
            motormovespeedby = 1.0;
       }
        rightMotor.set(motormovespeedby);
        SmartDashboard.putNumber("Motor Speed", motormovespeedby);
    }

    public void stopMotor(){
        rightMotor.stopMotor();
    }
}
