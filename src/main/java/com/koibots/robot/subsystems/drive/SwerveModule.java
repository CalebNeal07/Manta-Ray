// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.koibots.robot.Constants.CANDeviceIDs;
import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.util.SparkUtils;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashSet;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import monologue.Logged;

public class SwerveModule implements Logged {
    private final SparkPIDController turnPIDController;
    private final SparkAbsoluteEncoder turnAbsoluteEncoder;
    private final TalonFX driveTalon;
    private final MotionMagicVelocityVoltage driveOutputRequest =
            new MotionMagicVelocityVoltage(0, 0, false, 0, 0, false, false, false);

    @Getter @Setter private SwerveModuleState targetState;

    protected SwerveModule(int turnID, int driveID) {
        CANSparkMax turnSparkMax = new CANSparkMax(turnID, MotorType.kBrushless);
        turnPIDController = turnSparkMax.getPIDController();
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();

        // Restore defaults before configuring
        turnSparkMax.restoreFactoryDefaults();

        // REMINDER: Check that rotating the swerve wheel ccw increases the encoder value (ccw+).
        // Ensure every modules zero is pointing forward with the bevel gear pointed to the
        // robot's right side. Make sure gyro is also ccw+.
        if (turnSparkMax.enableVoltageCompensation(12) != REVLibError.kOk
                || turnSparkMax.setCANTimeout(50)
                        != REVLibError.kOk // Increase timeout during config
                || turnSparkMax.setIdleMode(IdleMode.kBrake) != REVLibError.kOk
                || turnSparkMax.setSecondaryCurrentLimit(60) != REVLibError.kOk
                || turnAbsoluteEncoder.setInverted(true) != REVLibError.kOk
                || turnAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI) != REVLibError.kOk
                || turnAbsoluteEncoder.setZeroOffset(
                                turnID == CANDeviceIDs.FL_TURN
                                        ? Math.PI / 4
                                        : turnID == CANDeviceIDs.FR_TURN
                                                ? 7 * Math.PI / 4
                                                : turnID == CANDeviceIDs.BL_TURN
                                                        ? 5 * Math.PI / 4
                                                        : 3 * Math.PI / 4)
                        != REVLibError.kOk
                || turnPIDController.setP(ControlConstants.TURN_KP, 0) != REVLibError.kOk
                || turnPIDController.setD(ControlConstants.TURN_KD, 0) != REVLibError.kOk
                || turnPIDController.setPositionPIDWrappingEnabled(true) != REVLibError.kOk
                || turnPIDController.setPositionPIDWrappingMinInput(0) != REVLibError.kOk
                || turnPIDController.setPositionPIDWrappingMinInput(2 * Math.PI) != REVLibError.kOk
                || turnPIDController.setFeedbackDevice(turnAbsoluteEncoder) == REVLibError.kOk) {

            DriverStation.reportError(
                    "Failed to write settings to" + SparkUtils.name(turnSparkMax), false);
        }

        turnSparkMax.setCANTimeout(20); // Restore default CAN Timeout

        // Configure SparkMAX to only send necesary data
        Set<SparkUtils.Data> data = new HashSet<>();
        Set<SparkUtils.Sensor> sensors = new HashSet<>();

        data.add(SparkUtils.Data.TEMPERATURE);
        data.add(SparkUtils.Data.CURRENT);
        data.add(SparkUtils.Data.POSITION);

        sensors.add(SparkUtils.Sensor.ABSOLUTE);

        SparkUtils.configureFrameStrategy(turnSparkMax, data, sensors, false);

        driveTalon = new TalonFX(driveID);

        TalonFXConfigurator driveMotorConfigurator = driveTalon.getConfigurator();
    }

    public void periodic() {
        turnPIDController.setReference(
                targetState.angle.getRadians(),
                CANSparkBase.ControlType.kPosition,
                0,
                ControlConstants.TURN_KS);

        driveTalon.setControl(driveOutputRequest);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                0, Rotation2d.fromRadians(turnAbsoluteEncoder.getPosition()));
    }

    public SwerveModuleState getRealState() {
        return new SwerveModuleState(
                driveTalon.getVelocity().getValue(),
                Rotation2d.fromRadians(turnAbsoluteEncoder.getPosition()));
    }
}
