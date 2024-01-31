// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrainConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
   private WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(DriveTrainConstants.LEFT_MOTOR_1_PORT);
   private WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(DriveTrainConstants.LEFT_MOTOR_2_PORT);
   
   private WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(DriveTrainConstants.RIGHT_MOTOR_1_PORT);
   private WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(DriveTrainConstants.RIGHT_MOTOR_2_PORT);

   private DifferentialDrive m_Drive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

  public DriveTrain() {
    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);

  
  }
  public void arcade ( double x, double y){
    m_Drive.arcadeDrive(x, y);
  }

  public Command hu (){
    return run(
      () -> {
      arcade(RobotContainer.m_drivercontroller.getLeftX()*.6 , (RobotContainer.m_drivercontroller.getLeftTriggerAxis() -RobotContainer.m_drivercontroller.getRightTriggerAxis()));
      }
  );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

}
