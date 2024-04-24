// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.koibots.robot.Constants.CANDeviceIDs;
import com.koibots.robot.Constants.PhysicalConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Monologue;

public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private final Pigeon2 pigeon2;
    
    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(PhysicalConstants.MODULE_LOCATIONS);

    public Swerve(Pigeon2 pigeon2) {
        frontLeftModule = new SwerveModule(CANDeviceIDs.FL_DRIVE, CANDeviceIDs.FL_TURN);
        frontRightModule = new SwerveModule(CANDeviceIDs.FR_DRIVE, CANDeviceIDs.FR_TURN);
        backLeftModule = new SwerveModule(CANDeviceIDs.BL_DRIVE, CANDeviceIDs.BL_TURN);
        backRightModule = new SwerveModule(CANDeviceIDs.BR_DRIVE, CANDeviceIDs.BR_TURN);

        this.pigeon2 = pigeon2;

        Monologue.logObj(frontLeftModule, "Swerve/Front-Left");
    }

    public Command teleopDriveCommand(
            DoubleSupplier xtranslationSupplier,
            DoubleSupplier yTranslationSupplier,
            DoubleSupplier rotSupplier) {
        return new Command() {
            SwerveModuleState[] prevSetpoints = new SwerveModuleState[4];
            ChassisSpeeds prevSpeeds = new ChassisSpeeds();
            double prevTime;
            
            @Override
            public void initialize() {
                prevSetpoints = new SwerveModuleState[] {
                    frontLeftModule.getRealState(),
                    frontRightModule.getRealState(),
                    backLeftModule.getRealState(),
                    backRightModule.getRealState()
                };
                
                prevTime = RobotController.getFPGATime();
            }

            @Override
            public void execute() {
                ExponentialProfile x = ;
                
                
                ChassisSpeeds desiredState = new ChassisSpeeds(); // TODO: Implement setpoint setting
                
                // TODO: Clean this up. Use actual vectors and change time from microseconds to seconds

                kinematics.toSwerveModuleStates(new ChassisSpeeds());

                SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredState);
 
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, DCMotor.getKrakenX60(1).withReduction(PhysicalConstants.DRIVE_GEAR_RATIO).freeSpeedRadPerSec);
                desiredState = kinematics.toChassisSpeeds(desiredModuleStates);

                // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so just use the previous angle.
                boolean need_to_steer = true;
                if (new Twist2d(desiredState.vxMetersPerSecond, desiredState.vyMetersPerSecond, desiredState.omegaRadiansPerSecond).equals(new Twist2d(0, 0, 0))) {
                    need_to_steer = false;
                    for (int i = 0; i < 4; ++i) {
                        desiredModuleStates[i].angle = prevSetpoints[i].angle;
                        desiredModuleStates[i].speedMetersPerSecond = 0.0;
                    }
                }

                // For each module, compute local Vx and Vy vectors.
                double[] prev_vx = new double[] {
                        prevSetpoints[0].angle.getCos() * prevSetpoints[0].speedMetersPerSecond,
                        prevSetpoints[1].angle.getCos() * prevSetpoints[1].speedMetersPerSecond,
                        prevSetpoints[2].angle.getCos() * prevSetpoints[2].speedMetersPerSecond,
                        prevSetpoints[3].angle.getCos() * prevSetpoints[3].speedMetersPerSecond
                };
                double[] prev_vy = new double[] {
                        prevSetpoints[0].angle.getSin() * prevSetpoints[0].speedMetersPerSecond,
                        prevSetpoints[1].angle.getSin() * prevSetpoints[1].speedMetersPerSecond,
                        prevSetpoints[2].angle.getSin() * prevSetpoints[2].speedMetersPerSecond,
                        prevSetpoints[3].angle.getSin() * prevSetpoints[3].speedMetersPerSecond
                };
                Rotation2d[] prev_heading = new Rotation2d[] {
                        prevSetpoints[0].speedMetersPerSecond < 0.0 ? prevSetpoints[0].angle.unaryMinus() : prevSetpoints[0].angle,
                        prevSetpoints[1].speedMetersPerSecond < 0.0 ? prevSetpoints[1].angle.unaryMinus() : prevSetpoints[1].angle,
                        prevSetpoints[2].speedMetersPerSecond < 0.0 ? prevSetpoints[2].angle.unaryMinus() : prevSetpoints[2].angle,
                        prevSetpoints[3].speedMetersPerSecond < 0.0 ? prevSetpoints[3].angle.unaryMinus() : prevSetpoints[3].angle
                };
                double[] desired_vx = new double[] {
                        desiredModuleStates[0].angle.getCos() * desiredModuleStates[0].speedMetersPerSecond,
                        desiredModuleStates[1].angle.getCos() * desiredModuleStates[1].speedMetersPerSecond,
                        desiredModuleStates[2].angle.getCos() * desiredModuleStates[2].speedMetersPerSecond,
                        desiredModuleStates[3].angle.getCos() * desiredModuleStates[3].speedMetersPerSecond
                };
                double[] desired_vy = new double[] {
                        desiredModuleStates[0].angle.getSin() * desiredModuleStates[0].speedMetersPerSecond,
                        desiredModuleStates[1].angle.getSin() * desiredModuleStates[1].speedMetersPerSecond,
                        desiredModuleStates[2].angle.getSin() * desiredModuleStates[2].speedMetersPerSecond,
                        desiredModuleStates[3].angle.getSin() * desiredModuleStates[3].speedMetersPerSecond
                };
                Rotation2d[] desired_heading = new Rotation2d[] {
                        desiredModuleStates[0].speedMetersPerSecond < 0.0 ? desiredModuleStates[0].angle.unaryMinus() : desiredModuleStates[0].angle,
                        desiredModuleStates[1].speedMetersPerSecond < 0.0 ? desiredModuleStates[1].angle.unaryMinus() : desiredModuleStates[1].angle,
                        desiredModuleStates[2].speedMetersPerSecond < 0.0 ? desiredModuleStates[2].angle.unaryMinus() : desiredModuleStates[2].angle,
                        desiredModuleStates[3].speedMetersPerSecond < 0.0 ? desiredModuleStates[3].angle.unaryMinus() : desiredModuleStates[3].angle
                };

                // Compute the deltas between start and goal. We can then interpolate from the start state to the goal state; then
                // find the amount we can move from start towards goal in this cycle such that no kinematic limit is exceeded.
                double dx = desiredState.vxMetersPerSecond - prevSpeeds.vxMetersPerSecond;
                double dy = desiredState.vyMetersPerSecond - prevSpeeds.vyMetersPerSecond;
                double dtheta = desiredState.omegaRadiansPerSecond - prevSpeeds.omegaRadiansPerSecond;

                // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at desiredState.
                double min_s = 1.0;

                // In cases where an individual module is stopped, we want to remember the right steering angle to command (since
                // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
                List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(4);
                // Enforce steering velocity limits. We do this by taking the derivative of steering angle at the current angle,
                // and then backing out the maximum interpolant between start and goal states. We remember the minimum across all modules, since
                // that is the active constraint.
                final double max_theta_step = (RobotController.getFPGATime() - prevTime) * DCMotor.getNEO(1).withReduction(PhysicalConstants.TURN_GEAR_RATIO).freeSpeedRadPerSec;
                for (int i = 0; i < 4; ++i) {
                    if (!need_to_steer) {
                        overrideSteering.add(Optional.of(prevSetpoints[i].angle));
                        continue;
                    }
                    overrideSteering.add(Optional.empty());
                    if (Math.abs(prevSetpoints[i].speedMetersPerSecond) < 1E-9) {
                        // If module is stopped, we know that we will need to move straight to the final steering angle, so limit based
                        // purely on rotation in place.
                        if (Math.abs(desiredModuleStates[i].speedMetersPerSecond) < 1E-9) {
                            // Goal angle doesn't matter. Just leave module at its current angle.
                            overrideSteering.set(i, Optional.of(prevSetpoints[i].angle));

                            continue;
                        }

                        var necessaryRotation = prevSetpoints[i].angle.unaryMinus().rotateBy(
                                desiredModuleStates[i].angle);
                        if (Math.abs(necessaryRotation.getRadians()) > Math.PI / 2.0) {
                            necessaryRotation = necessaryRotation.unaryMinus();
                        }
                        // getRadians() bounds to +/- Pi.
                        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

                        if (numStepsNeeded <= 1.0) {
                            // Steer directly to goal angle.
                            overrideSteering.set(i, Optional.of(desiredModuleStates[i].angle));
                            // Don't limit the global min_s;
                            continue;
                        } else {
                            // Adjust steering by max_theta_step.
                            overrideSteering.set(i, Optional.of(prevSetpoints[i].angle.rotateBy(
                                    Rotation2d.fromRadians(Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
                            min_s = 0.0;
                            continue;
                        }
                    }
                    if (min_s == 0.0) {
                        // s can't get any lower. Save some CPU.
                        continue;
                    }

                    final int kMaxIterations = 8;
                    double s = findSteeringMaxS(prev_vx[i], prev_vy[i], prev_heading[i].getRadians(),
                                                desired_vx[i], desired_vy[i], desired_heading[i].getRadians(),
                                                max_theta_step);
                    min_s = Math.min(min_s, s);
                }

                // Enforce drive wheel acceleration limits.
                final double max_vel_step = (RobotController.getFPGATime() - prevTime) * PhysicalConstants.MAX_ACCEL;
                for (int i = 0; i < 4; ++i) {
                    if (min_s == 0.0) {
                        // No need to carry on.
                        break;
                    }
                    double vx_min_s = min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
                    double vy_min_s = min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
                    // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we already know we can't go faster
                    // than that.
                    final int kMaxIterations = 10;
                    double s = min_s * findDriveMaxS(prev_vx[i], prev_vy[i], Math.hypot(prev_vx[i], prev_vy[i]),
                                                     vx_min_s, vy_min_s, Math.hypot(vx_min_s, vy_min_s),
                                                     max_vel_step);
                    min_s = Math.min(min_s, s);
                }

                ChassisSpeeds retSpeeds = new ChassisSpeeds(
                        prevSpeeds.vxMetersPerSecond + min_s * dx,
                        prevSpeeds.vyMetersPerSecond + min_s * dy,
                        prevSpeeds.omegaRadiansPerSecond + min_s * dtheta);
                var retStates = kinematics.toSwerveModuleStates(retSpeeds);
                for (int i = 0; i < 4; ++i) {
                    final var maybeOverride = overrideSteering.get(i);
                    if (maybeOverride.isPresent()) {
                        var override = maybeOverride.get();
                        if (Math.abs(retStates[i].angle.unaryMinus().rotateBy(override).getRadians()) > Math.PI / 2.0) {
                            retStates[i].speedMetersPerSecond *= -1.0;
                        }
                        retStates[i].angle = override;
                    }
                    final var deltaRotation = prevSetpoints[i].angle.unaryMinus().rotateBy(retStates[i].angle);
                    if (Math.abs(deltaRotation.getRadians()) > Math.PI / 2) {
                        retStates[i].angle = retStates[i].angle.unaryMinus();
                        retStates[i].speedMetersPerSecond *= -1.0;
                    }
                }
                
                frontLeftModule.setTargetState(retStates[0]);
                frontRightModule.setTargetState(retStates[1]);
                backLeftModule.setTargetState(retStates[2]);
                backRightModule.setTargetState(retStates[3]);
                
                prevSetpoints = retStates;
                prevSpeeds = desiredState;
                prevTime = RobotController.getFPGATime();
            }
        };
    }

    public Command autonomousPathFollowingCommand() {
        return new Command() {};
    }

    @FunctionalInterface
    private interface Function2d {
        double f(double x, double y);
    }


    /**
    * Find the root of the generic 2D parametric function 'func' using the regula falsi technique. This is a pretty naive way to
    * do root finding, but it's usually faster than simple bisection while being robust in ways that e.g. the Newton-Raphson
    * method isn't.
    * @param func The Function2d to take the root of.
    * @param x_0 x value of the lower bracket.
    * @param y_0 y value of the lower bracket.
    * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during recursion)
    * @param x_1 x value of the upper bracket.
    * @param y_1 y value of the upper bracket.
    * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during recursion)
    * @param iterations_left Number of iterations of root finding left.
    * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the (approximate) root.
    */
    private double findRoot(Function2d func, double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, int iterations_left) {
        if (iterations_left < 0 || Math.abs(f_1 - f_0) < 1E-9) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess + (1.0 - s_guess) * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    protected double findSteeringMaxS(double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_deviation) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func = (x,y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, 8);
    }

    protected double findDriveMaxS(double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_vel_step) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func = (x,y) -> Math.hypot(x, y) - offset;

        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, 10);
    }

    private double unwrapAngle(double ref, double angle) {
            double diff = angle - ref;
            if (diff > Math.PI) {
                return angle - 2.0 * Math.PI;
            } else if (diff < -Math.PI) {
                return angle + 2.0 * Math.PI;
            } else {
                return angle;
            }
        }
}
