package frc.robot.swerveHelpers;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;

import frc.robot.Constants;

public class CtreSettings {

    public static void UpdateCanEncoderDefaultSettings()
    {
        CANCoder backLeftCoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
        CANCoder backRightCoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);
        CANCoder frontLeftCoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
        CANCoder frontRightCoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backRightCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontRightCoder);
    }
    
    private static void UpdateSingleCanEncoderDefaultSettings(CANCoder cancoder)
    {
        CANCoderConfiguration config = new CANCoderConfiguration();
        SensorInitializationStrategy strat = SensorInitializationStrategy.BootToAbsolutePosition;
        config.initializationStrategy = strat;
        cancoder.configAllSettings(config);
    }
}
