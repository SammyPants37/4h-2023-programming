package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;

import java.util.function.DoubleSupplier;

public class claw extends SubsystemBase {

    private final CANSparkMax leftClaw;
    private final CANSparkMax rightClaw;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final PIDController leftController;
    private final PIDController rightController;

    public claw() {
        leftClaw = new CANSparkMax(clawConstants.leftMotor, MotorType.kBrushless);
        rightClaw = new CANSparkMax(clawConstants.rightMotor, MotorType.kBrushless);

        leftEncoder = leftClaw.getEncoder();
        rightEncoder = rightClaw.getEncoder();

        leftController = new PIDController(
                clawConstants.kp,
                clawConstants.ki,
                clawConstants.kd);
        leftController.setTolerance(0.5);
        rightController = new PIDController(
                clawConstants.kp,
                clawConstants.ki,
                clawConstants.kd);
        rightController.setTolerance(0.5);

        configureMotors();
    }

    private void configureMotors() {
        leftClaw.setIdleMode(IdleMode.kBrake);
        rightClaw.setIdleMode(IdleMode.kBrake);

        leftClaw.setInverted(false);
        rightClaw.setInverted(false);

        leftClaw.setSmartCurrentLimit(20);
        rightClaw.setSmartCurrentLimit(20);

        leftClaw.setSoftLimit(SoftLimitDirection.kForward, clawConstants.leftSoftLimitRots);
        leftClaw.enableSoftLimit(SoftLimitDirection.kForward, true);
        leftClaw.enableSoftLimit(SoftLimitDirection.kReverse, false);

        rightClaw.setSoftLimit(SoftLimitDirection.kForward, clawConstants.rightSoftLimitRots);
        rightClaw.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightClaw.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    // ACTIONS

    public void stopLeft() {
        leftClaw.stopMotor();
    }

    public void stopRight() {
        rightClaw.stopMotor();
    }

    public void setLeft(double speed) {
        leftClaw.set(speed);
    }

    public void setRight(double speed) {
        rightClaw.set(speed);
    }

    public void goToTargetPositionLeft() {
        final double pos = leftEncoder.getPosition();
        final double speed = leftController.calculate(pos);
        setLeft(speed);
    }

    public void goToTargetPositionRight() {
        final double pos = rightEncoder.getPosition();
        final double speed = rightController.calculate(pos);
        setRight(speed);
    }

    public void setTargetPositionLeft(double position) {
        leftController.reset();
        leftController.setSetpoint(position);
    }

    public void setTargetPositionRight(double position) {
        rightController.reset();
        rightController.setSetpoint(position);
    }

    // GETTERS

    public double getLeftPos() {
        return leftEncoder.getPosition();
    }

    public double getRightPos() {
        return rightEncoder.getPosition();
    }

    // STATES

    public boolean leftStopped() {
        return leftEncoder.getVelocity() < 0.1;
    }

    public boolean rightStopped() {
        return rightEncoder.getVelocity() < 0.1;
    }

    public boolean leftAtTargetPosition() {
        return leftController.atSetpoint();
    }

    public boolean rightAtTargetPosition() {
        return rightController.atSetpoint();
    }

    // COMMANDS

    public CommandBase stopLeftCommand() {
        return this.run(this::stopLeft);
    }

    public CommandBase stopRightCommand() {
        return this.run(this::stopRight);
    }

    public CommandBase runLeftWithSpeed(double speed) {
        return this.run(() -> setLeft(speed));
    }

    public CommandBase runRightWithSpeed(double speed) {
        return this.run(() -> setRight(speed));
    }

    public CommandBase runwithJoysticks(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        return new ParallelCommandGroup(
                this.run(() -> setLeft(MathUtil.applyDeadband(leftSpeed.getAsDouble(), 0.05))),
                this.run(() -> setRight(MathUtil.applyDeadband(rightSpeed.getAsDouble(), 0.02))));
    }

    public CommandBase leftGoToPosition(double pos) {
        return this.runOnce(() -> setTargetPositionLeft(pos*clawConstants.kInches2Rots))
                .andThen(this::goToTargetPositionLeft).until(this::leftAtTargetPosition);
    }

    public CommandBase rightGoToPosition(double pos) {
        return this.runOnce(() -> setTargetPositionRight(pos*clawConstants.kInches2Rots))
                .andThen(this::goToTargetPositionRight).until(this::rightAtTargetPosition);
    }
}

