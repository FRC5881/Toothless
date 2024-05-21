// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  // If you choose to move the Field2d object be careful to not create a new one every loop.
  //
  // There should only be 1 Field2d object per robot.
  Field2d m_field = new Field2d();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Update all simulated components - DO NOT REMOVE
    double dt = 0.02;
    Simulation.drivetrainSim.update(dt);
    Simulation.intakeSim.update(dt);
    Simulation.launcherSim.update(dt);
    Simulation.indexerSim.update(dt);
    Simulation.agitatorSim.update(dt);
    Simulation.climberSim.update(dt);
    
    // EXAMPLE USAGE - YOU PROBABLY WANT TO REMOVE
    Simulation.rampSim.set(Value.kForward);
    SmartDashboard.putBoolean("Ramp", Simulation.rampSim.get() == Value.kForward);
    
    Simulation.drivetrainSim.setInputs(2, 2.1);
    SmartDashboard.putNumber("Left Encoder", Simulation.drivetrainSim.getLeftPositionMeters());
    SmartDashboard.putNumber("Right Encoder", Simulation.drivetrainSim.getRightPositionMeters());

    m_field.setRobotPose(Simulation.drivetrainSim.getPose());
    SmartDashboard.putData(m_field);

    Simulation.intakeSim.setInputVoltage(12);
    SmartDashboard.putNumber("Intake Velocity", Simulation.intakeSim.getAngularVelocityRPM());

    Simulation.launcherSim.setInputVoltage(12);
    SmartDashboard.putNumber("Launcher Velocity", Simulation.launcherSim.getAngularVelocityRPM());

    Simulation.indexerSim.setInputVoltage(12);
    SmartDashboard.putNumber("Indexer Velocity", Simulation.indexerSim.getAngularVelocityRPM());

    Simulation.agitatorSim.setInputVoltage(12);
    SmartDashboard.putNumber("Agitator Velocity", Simulation.agitatorSim.getAngularVelocityRPM());

    Simulation.climberSim.setInputVoltage(12);
    SmartDashboard.putNumber("Climber Velocity", Simulation.climberSim.getPositionMeters());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
