package frc.robot;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Simulation {
    // Drivetrain Simulation
    private static final DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide, DifferentialDrivetrainSim.KitbotGearing.k10p71, DifferentialDrivetrainSim.KitbotWheelSize.kSixInch, null);

    public static final MotorController leftDriveMotor = new PWMSparkMax(0);
    public static final MotorController rightDriveMotor = new PWMSparkMax(1);
    
    private static final Encoder leftSimEncoder = new Encoder(0, 1);
    public static final EncoderSim leftDriveEncoder = new EncoderSim(leftSimEncoder);

    private static final Encoder rightSimEncoder = new Encoder(2, 3);
    public static final EncoderSim rightDriveEncoder = new EncoderSim(rightSimEncoder);

    private static double updateDrivetrainSim() {
        double voltage = RobotController.getBatteryVoltage();
        drivetrainSim.setInputs(leftDriveMotor.get() * voltage, rightDriveMotor.get() * voltage);

        SmartDashboard.putNumber("/Simulation/Left Voltage", leftDriveMotor.get() * voltage);
        SmartDashboard.putNumber("/Simulation/Right Voltage", rightDriveMotor.get() * voltage);

        drivetrainSim.update(0.02);

        leftDriveEncoder.setDistance(drivetrainSim.getLeftPositionMeters());
        leftDriveEncoder.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());

        rightDriveEncoder.setDistance(drivetrainSim.getRightPositionMeters());
        rightDriveEncoder.setRate(drivetrainSim.getRightVelocityMetersPerSecond());

        return drivetrainSim.getCurrentDrawAmps();
    }

    // Intake Simulation
    private static final FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNeo550(1), 15.0, 0.1);
    public static final MotorController intakeMotor = new PWMSparkMax(2);

    private static double updateIntakeSim() {
        double voltage = RobotController.getBatteryVoltage();
        intakeSim.setInput(intakeMotor.get() * voltage);

        SmartDashboard.putNumber("/Simulation/Intake Voltage", intakeMotor.get() * voltage);

        intakeSim.update(0.02);

        return intakeSim.getCurrentDrawAmps();
    }
    
    // Launcher Simulation
    private static final FlywheelSim launcherSim = new FlywheelSim(DCMotor.getNEO(2), 1.0, 0.005);
    public static final MotorController launcherMotor = new PWMSparkMax(3);

    private static double updateLauncherSim() {
        double voltage = RobotController.getBatteryVoltage();
        launcherSim.setInput(launcherMotor.get() * voltage);

        SmartDashboard.putNumber("/Simulation/Launcher Voltage", launcherMotor.get() * voltage);

        launcherSim.update(0.02);

        return launcherSim.getCurrentDrawAmps();
    }

    // Indexer Simulation
    private static final FlywheelSim indexerSim = new FlywheelSim(DCMotor.getNeo550(1), 5.0, 0.1);
    public static final MotorController indexerMotor = new PWMSparkMax(4);

    private static double updateIndexerSim() {
        double voltage = RobotController.getBatteryVoltage();
        indexerSim.setInput(indexerMotor.get() * voltage);

        SmartDashboard.putNumber("/Simulation/Indexer Voltage", indexerMotor.get() * voltage);

        indexerSim.update(0.02);

        return indexerSim.getCurrentDrawAmps();
    }

    // Agitator Simulation
    private static final FlywheelSim agitatorSim = new FlywheelSim(DCMotor.getNeo550(1), 5.0, 0.1);
    public static final MotorController agitatorMotor = new PWMSparkMax(5);

    private static double updateAgitatorSim() {
        double voltage = RobotController.getBatteryVoltage();
        agitatorSim.setInput(agitatorMotor.get() * voltage);

        SmartDashboard.putNumber("/Simulation/Agitator Voltage", agitatorMotor.get() * voltage);

        agitatorSim.update(0.02);

        return agitatorSim.getCurrentDrawAmps();
    }

    // Climber Simulation
    private static final ElevatorSim climberSim = new ElevatorSim(DCMotor.getNEO(2), 50.0, 50.0, 0.0254, 0.0, 2.0, true, 0.0);
    public static final MotorController climberMotor = new PWMSparkMax(6);
    public static final EncoderSim climberEncoder = new EncoderSim(new Encoder(4, 5));

    private static double updateClimberSim() {
        double voltage = RobotController.getBatteryVoltage();
        climberSim.setInput(climberMotor.get() * voltage);
        
        SmartDashboard.putNumber("/Simulation/Climber Voltage", climberMotor.get() * voltage);

        climberSim.update(0.02);

        climberEncoder.setDistance(climberSim.getPositionMeters());
        climberEncoder.setRate(climberSim.getVelocityMetersPerSecond());

        return climberSim.getCurrentDrawAmps();
    }

    // Ramp Simulation
    public static final DoubleSolenoidSim rampSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);

    // Full System Simulation
    public static void run() {
        double drivetrainCurrent = updateDrivetrainSim();
        SmartDashboard.putNumber("/Simulation/Drivetrain Current", drivetrainCurrent);

        double intakeCurrent = updateIntakeSim();
        SmartDashboard.putNumber("/Simulation/Intake Current", intakeCurrent);

        double launcherCurrent = updateLauncherSim();
        SmartDashboard.putNumber("/Simulation/Launcher Current", launcherCurrent);

        double indexerCurrent = updateIndexerSim();
        SmartDashboard.putNumber("/Simulation/Indexer Current", indexerCurrent);

        double agitatorCurrent = updateAgitatorSim();
        SmartDashboard.putNumber("/Simulation/Agitator Current", agitatorCurrent);

        double climberCurrent = updateClimberSim();
        SmartDashboard.putNumber("/Simulation/Climber Current", climberCurrent);

        double totalCurrent = drivetrainCurrent + intakeCurrent + launcherCurrent + indexerCurrent + agitatorCurrent + climberCurrent;
        SmartDashboard.putNumber("/Simulation/Total Current", totalCurrent);

        double voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent);
        SmartDashboard.putNumber("/Simulation/Battery Voltage", voltage);

        RoboRioSim.setVInVoltage(voltage);
    }
}
