package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

import java.util.function.DoubleSupplier;

public class elevator extends SubsystemBase {

    private final CANSparkMax leaderLeft;
    private final CANSparkMax followerRight;

    private final RelativeEncoder encoder;

    private final PIDController controller;

    public elevator() {
        leaderLeft = new CANSparkMax(elevatorConstants.LeftElevatorMotor, MotorType.kBrushless);
        followerRight = new CANSparkMax(elevatorConstants.RightElevatorMotor, MotorType.kBrushless);

        encoder = leaderLeft.getEncoder();

        controller = new PIDController(
                elevatorConstants.kp,
                elevatorConstants.ki,
                elevatorConstants.kd);
        controller.setTolerance(1);

        configureMotors();
    }

    private void configureMotors() {
        leaderLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        followerRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leaderLeft.setInverted(false);

        followerRight.follow(leaderLeft, true);

        leaderLeft.setSmartCurrentLimit(60);
        followerRight.setSmartCurrentLimit(60);

        leaderLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
        leaderLeft.setSoftLimit(SoftLimitDirection.kForward, 84);
        leaderLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leaderLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    // ACTIONS

    public void stop() {
        leaderLeft.stopMotor();
    }

    public void set(double speed) {
        leaderLeft.set(speed);
    }

    public void goToTargetHeight() {
        final double angle = getPosition();
        final double speed = controller.calculate(angle);
        set(speed);
    }

    public void setTargetHeight(double height) {
        controller.reset();
        controller.setSetpoint(height);
    }

    // GETTERS

    public double getPosition() {
        return encoder.getPosition();
    }

    // STATES

    public boolean stopped() {
        return encoder.getVelocity() <= 0.1;
    }

    public boolean atTargetHeight() {
        return controller.atSetpoint();
    }

    // COMMANDS

    public CommandBase stopCommand() {
        return this.runOnce(this::stop);
    }

    public CommandBase runWithSpeed(double speed) {
        return this.run(() -> set(speed));
    }

    public CommandBase runWithJoysticks(DoubleSupplier speed) {
        return this.run(() -> set(MathUtil.applyDeadband(speed.getAsDouble(), 0.02)));
    }

    public CommandBase goToHeight(double height) {
        return this.runOnce(() -> setTargetHeight(height))
                .andThen(this::goToTargetHeight).until(this::atTargetHeight);
    }
}
