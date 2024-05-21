package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class Simulation {
    /**
     * Drivetrain simulation
     */
    public static final DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide, DifferentialDrivetrainSim.KitbotGearing.k10p71, DifferentialDrivetrainSim.KitbotWheelSize.kSixInch, null);

    /**
     * Intake simulation
     */
    public static final FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNeo550(1), 15.0, 0.1);

    /**
     * Launcher simulation
     */
    public static final FlywheelSim launcherSim = new FlywheelSim(DCMotor.getNEO(2), 1.0, 0.005);

    /**
     * Indexer simulation, feeds FUEL from intake to launcher
     */
    public static final FlywheelSim indexerSim = new FlywheelSim(DCMotor.getNeo550(1), 5.0, 0.1);

    /**
     * Agitator simulation, it just spins the FUEL around
     */
    public static final FlywheelSim agitatorSim = new FlywheelSim(DCMotor.getNeo550(1), 5.0, 0.1);

    /**
     * Pneumatic ramp/shield simulation. kForward creates a ramp for FUEL, kReverse retracts the ramp and allows the robot to collect GEARS
     */
    public static final DoubleSolenoidSim rampSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);

    /**
     * Climber simulation, much heavier mechanism than the other components
     */
    public static final ElevatorSim climberSim = new ElevatorSim(DCMotor.getNEO(2), 50.0, 50.0, 0.0254, 0.0, 2.0, true, 0.0);
}
