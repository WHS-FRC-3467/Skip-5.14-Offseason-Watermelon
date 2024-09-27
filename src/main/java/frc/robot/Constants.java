package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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

    public static class FieldConstants{
        // Assume that Blue is the Origin for everything. See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#rotation-conventions
        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.62, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.62, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5),Units.inchesToMeters(323.00),new Rotation2d(Math.PI/2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline

        public static final Rotation2d ampAngle = new Rotation2d(Math.PI / 2);
    }
}