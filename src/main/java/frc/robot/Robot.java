// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;

  private final XboxController m_Xbox;

  private final TalonFX m_rightMotor1 = new TalonFX(0);
  private final TalonFX m_leftMotor2 = new TalonFX(1);
  private final TalonFX m_rightMotor3 = new TalonFX(2);
  private final TalonFX m_leftMotor4 = new TalonFX(3);
  private final TalonFX m_rightMotor5 = new TalonFX(4);
  private final TalonFX m_leftMotor6 = new TalonFX(5);


  private final Follower m_rightFollower = new Follower(0, false);
  private final Follower m_leftFollower = new Follower(1, false);
  

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    m_rightMotor1.setInverted(true);

    //joystick right motors right Y joystick, left motors left Y joystick

    m_rightMotor3.setControl(m_rightFollower);
    m_leftMotor4.setControl(m_leftFollower);
    m_rightMotor5.setControl(m_rightFollower);
    m_leftMotor6.setControl(m_leftFollower);

    m_robotDrive = new DifferentialDrive(m_rightMotor1::set, m_leftMotor2::set);
    
    //manette xbox
    m_Xbox = new XboxController(0);
    
    SendableRegistry.addChild(m_robotDrive, m_rightMotor1);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor2);


    SmartDashboard.putData("DifferentialDrive", new Sendable() {
    @Override
  
  
    public void initSendable(SendableBuilder builder) {
        //drag and drop differential drive
         builder.setSmartDashboardType("DifferentialDrive");
    
          builder.addDoubleProperty("Right Motor Speed", () -> m_rightMotor1.getMotorVoltage().getValueAsDouble(), null);
          builder.addDoubleProperty("Left Motor Speed", () -> m_leftMotor2.getMotorVoltage().getValueAsDouble(), null);
        
          //builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians().getValueAsDouble(), null);
        }
      });
  }

  
  @Override
  public void teleopPeriodic() {
    //Right and Back are positive
    m_robotDrive.tankDrive(m_Xbox.getLeftTriggerAxis(), m_Xbox.getRightTriggerAxis());
    //m_robotDrive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
  }
}
