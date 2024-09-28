package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class Constants {
    
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class DriveConstants {
        public static final double headingAngleTolerance = 2.0;
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.CurrentLimits.SupplyCurrentLimit = 30;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 90;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.01;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 80;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }
    
    public static final class ShooterJointConstants {
        public static final int ID_SHOOTER_JOINT = 19; //Made Up For Now
        public static final double FlywheelVelocityTolerance = 5.0;
        
        public static TalonFXConfiguration shooterMotorConfig(int deviceID) {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();
    
            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Need to test this
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0; 
            m_configuration.CurrentLimits.StatorCurrentLimit = 60;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;
            m_configuration.CurrentLimits.SupplyCurrentLimit = 30;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 85;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
    
                // set up slot 0 gains - for Velocity PID - NOT TESTED
                m_configuration.Slot0.kS = 0.1;
                m_configuration.Slot0.kV = 0.125;
                m_configuration.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
                m_configuration.Slot0.kP = 2.1;
                m_configuration.Slot0.kI = 0;
                m_configuration.Slot0.kD = 0;
    
                // set Motion Magic settings
                    /* Actual values have yet to be tested */
                m_configuration.MotionMagic.MotionMagicCruiseVelocity = 8; // Target cruise velocity of 80 rps
                m_configuration.MotionMagic.MotionMagicAcceleration = 16; // Target acceleration of 16 rps/s
                m_configuration.MotionMagic.MotionMagicJerk = 160; // Target jerk of 160 rps/s/s

            return m_configuration;
        }

        
    }

    public static final class ShooterRollersConstants {
        public static final int ID_Flywheel = 21;
        public static final int ID_FlywheelFollower = 22;
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

        public static final class YSplitRollersConstants {
        public static final int ID_YSPLIT_ROLLER1 = 0;
        public static final int ID_YSPLIT_ROLLER2 = 1;

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

    public static class FieldConstants {

        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12),Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12),Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.62, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.62, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00),new Rotation2d(Math.PI / 2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00),new Rotation2d(-Math.PI / 2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline

        public static final Rotation2d ampAngle = new Rotation2d(Math.PI / 2);
    }
    
}
