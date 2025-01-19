package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim {
    //3 dcmotor sims

    DCMotorSim center = new DCMotorSim(new LinearSystem<N2, N1, N2>(new Matrix<R,C>(0)), new DCMotor(0.0, 0.0, 0.0, 0.0, 0.0, 0), 0.0);
}
