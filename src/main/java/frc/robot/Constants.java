package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    
    public static final class ShooterJointConstants {
        public static final int ID_SHOOTER_JOINT = 19; //Made Up For Now
        public static final double ShooterJointTolerance = 5.0;
        

        

        
    }

    public static final class ExampleSubsystemConstants {
            public static final int ID_Motor = 0;
    
            public static TalonFXConfiguration motorConfig() {
                TalonFXConfiguration m_configuration = new TalonFXConfiguration();
    
                m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                m_configuration.Voltage.PeakForwardVoltage = 12.0;
                m_configuration.Voltage.PeakReverseVoltage = -12.0;
    
                m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
                m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
                m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
                m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
                m_configuration.CurrentLimits.StatorCurrentLimit = 70;
                m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    
                return m_configuration;
            }
        }
    
}