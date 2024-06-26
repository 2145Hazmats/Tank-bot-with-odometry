// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  DifferentialDrive m_differentialDrive;
  DifferentialDriveKinematics m_driveKinematics;
  DifferentialDrivePoseEstimator m_driveOdometry;

  CANSparkMax leftMotor = new CANSparkMax(DrivetrainConstants.MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(DrivetrainConstants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  private RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final Field2d m_field = new Field2d();

  public Drivetrain() {
    rightMotorEncoder.setPositionConversionFactor(DrivetrainConstants.ENCODER_CONVERSION_FACTOR);
    leftMotorEncoder.setPositionConversionFactor(DrivetrainConstants.ENCODER_CONVERSION_FACTOR);
    rightMotorEncoder.setPosition(0);
    leftMotorEncoder.setPosition(0);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    gyro.calibrate();
    gyro.reset();

    m_differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
    m_driveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH);
    m_driveOdometry = new DifferentialDrivePoseEstimator(
        m_driveKinematics,
        new Rotation2d(gyro.getAngle()),
        0,
        0,
        new Pose2d()
    );
    // We can implement the standard deviation of vision to make pose estimation more accurate using a Matrix<N3, N1>, but it is complicated.

    SmartDashboard.putData("Field", m_field);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public Pose2d getPose2d() {
    return m_driveOdometry.getEstimatedPosition();
  }
  
  public Command drive(DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY, Supplier<String> driveType) {
    return Commands.run(() -> {
      switch (driveType.get()) {
        case "arcadeDrive":
          m_differentialDrive.arcadeDrive(leftY.getAsDouble(), rightX.getAsDouble());
          break;
        case "tankDrive":
          m_differentialDrive.tankDrive(leftY.getAsDouble(), rightY.getAsDouble());
          break;
        case "curvatureDrive1":
          m_differentialDrive.curvatureDrive(leftY.getAsDouble(), rightX.getAsDouble(), false);
          break;
        case "curvatureDrive2":
          m_differentialDrive.curvatureDrive(leftY.getAsDouble(), rightX.getAsDouble(), true);
          break;
      }
      SmartDashboard.putNumber("Controller Left Y", leftY.getAsDouble());
      SmartDashboard.putNumber("Controller Right X", rightX.getAsDouble());
      SmartDashboard.putNumber("Controller Right Y", rightY.getAsDouble());
      SmartDashboard.putString("Selected Drive Type", driveType.get());
    }, this);
  }

  public void resetOdometry() {
    // we can add this method, but m_driveOdometry.reset() wasn't working
  }

  public void addVisionPose2d(Pose2d visionPose2d) {
    m_driveOdometry.addVisionMeasurement(visionPose2d, Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {
    m_driveOdometry.updateWithTime(
        Timer.getFPGATimestamp(),
        new Rotation2d(Units.degreesToRadians(gyro.getAngle())),
        -leftMotorEncoder.getPosition(),
        -rightMotorEncoder.getPosition()
    );
    
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Bot X", getPose2d().getX());
    SmartDashboard.putNumber("Bot Y", getPose2d().getY());
    m_field.setRobotPose(m_driveOdometry.getEstimatedPosition());
  }
}
