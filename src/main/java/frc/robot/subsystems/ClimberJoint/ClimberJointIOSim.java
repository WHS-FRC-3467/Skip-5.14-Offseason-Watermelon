package frc.robot.subsystems.ClimberJoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberJointIOSim implements ClimberJointIO {
    private SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 0, 0, 0, 0, 0, false, 0);
    private PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ClimberJointIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = 
                MathUtil.clamp(pid.calculate(sim.getVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.positionRad = 0.0;
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        closedLoop = true;
        pid.setSetpoint(velocityRadPerSec);
        this.ffVolts = ffVolts;
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }
}
