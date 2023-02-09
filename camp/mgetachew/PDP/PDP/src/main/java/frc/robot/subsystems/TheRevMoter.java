package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TheRevMoter {
    /**
     *
     */
    private static WPI_TalonFX rightMotor = new WPI_TalonFX(16);
    private static double motormovespeedby = 0.0;
    private ShuffleboardTab tab = Shuffleboard.getTab("Right Motor");
   // stop causing errors - private NetworkTableEntry rightMotorSpeed = tab.add("Right Motor", 0).withWidget(BuiltInWidgets.kDial).getEntry();

    private TheRevMoter() {
        // put the default value

    }

    public static void driveTheMotor() { 
        rightMotor.set(motormovespeedby); 
    }

    public void setMotorSpeedFromDashboard(){

    }

    public void stopMotor(){
        rightMotor.stopMotor();
    }

    // from https://www.chiefdelphi.com/t/smartdashboard-input-output/162502/4 100% not plagarism
    private static double createSmartDashboardNumber(String key, double defValue) {

        // See if already on dashboard, and if so, fetch current value
        double value = SmartDashboard.getNumber(key, defValue);
      
        // Make sure value is on dashboard, puts back current value if already set
        // otherwise puts back default value
        SmartDashboard.putNumber(key, value);
      
        return value;
      }
      

}