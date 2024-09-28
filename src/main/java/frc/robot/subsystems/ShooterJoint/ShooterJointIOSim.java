package frc.robot.subsystems.ShooterJoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
// import the simulation class you want from WPI
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterJointIOSim implements ShooterJointIO {
    // establish the sim
    // establish the PID
    private PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    // Now we are going to copy over what we have in SimpleSubsystemIO methods and override them
    @Override
    public void updateInputs(ShooterJointIOInputs inputs) {
        if (closedLoop) {
            // this is where you can apply closed loop code
        }

        // You can establish how fast you want the sim to update
        // use a simulation class, see advantagekit example
        //sim.update(0.02);

        // Update the IO layer to the values you are now using
        inputs.test = 0.1;
        //inputs.position = sim.position;
        //inputs.velocity = sim.velocity;

    }

    @Override 
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
        // sim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        pid.setPID(kP, kI, kD);
    }

}