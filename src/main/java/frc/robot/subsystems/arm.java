package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

import java.util.function.DoubleSupplier;

public class arm extends SubsystemBase {

    private final CANSparkMax armMotor;

    private final RelativeEncoder encoder;

    private final PIDController controller;

    public arm() {
        armMotor = new CANSparkMax(armConstants.armMotor, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        controller = new PIDController(
                armConstants.kp,
                armConstants.ki,
                armConstants.kd);
        controller.setTolerance(1);

        configureMotors();
    }

    private void configureMotors() {
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.setInverted(false);

        armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, 58);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        armMotor.setSmartCurrentLimit(60);
    }

    // ACTIONS

    public void stop() {
        armMotor.stopMotor();
    }

    public void set(double speed) {
        armMotor.set(speed);
    }

    public void goToTargetLength() {
        final double angle = getPosition();
        final double speed = controller.calculate(angle);
        set(speed);
    }

    public void setTargetLength(double length) {
        controller.reset();
        controller.setSetpoint(length);
    }

    // GETTERS

    public double getPosition() {
        return encoder.getPosition();
    }

    // STATES

    public boolean stopped() {
        return encoder.getVelocity() <= 0.1;
    }

    public boolean atTargetLength() {
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

    public CommandBase goToLength(double length) {
        return this.runOnce(() -> setTargetLength(length))
                .andThen(this::goToTargetLength).until(this::atTargetLength);
    }
}

