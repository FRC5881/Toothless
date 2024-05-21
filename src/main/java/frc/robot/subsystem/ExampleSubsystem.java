package frc.robot.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Simulation;

public class ExampleSubsystem implements Subsystem {
    private static MotorController intakeMotor = Simulation.intakeMotor;

    public ExampleSubsystem() {
    }

    public Command cRun() {
        return runEnd(() -> {
            intakeMotor.set(0.5);
        }, intakeMotor::stopMotor);
    }
}
