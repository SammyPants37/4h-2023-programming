package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The Drive subsystem.
 */
public class drive extends SubsystemBase {
    private final AHRS gyro;

    private final WPI_TalonFX frontLeftMotor;
    private final WPI_TalonFX backLeftMotor;
    private final WPI_TalonFX frontRightMotor;
    private final WPI_TalonFX backRightMotor;

    private final DifferentialDrive drive;

    private final PIDController turningPIDController;
    private final PIDController drivePIDController;
    private final PIDController balancePIDController;

    public drive() {
        gyro = new AHRS(SPI.Port.kMXP);
        frontLeftMotor = new WPI_TalonFX(driveConstants.frontLeftPort);
        backLeftMotor = new WPI_TalonFX(driveConstants.backLeftPort);
        frontRightMotor = new WPI_TalonFX(driveConstants.frontRightPort);
        backRightMotor = new WPI_TalonFX(driveConstants.backRightPort);

        drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        drive.setDeadband(0.02);
        drive.setSafetyEnabled(true);


        turningPIDController = new PIDController(
                driveConstants.k_turnP,
                driveConstants.k_turnI,
                driveConstants.k_turnD);
        drivePIDController = new PIDController(
                driveConstants.k_driveP,
                driveConstants.k_driveI,
                driveConstants.k_driveD);
        balancePIDController = new PIDController(
                driveConstants.k_balanceP,
                driveConstants.k_balanceI,
                driveConstants.k_balanceD);

        turningPIDController.setTolerance(1);
        drivePIDController.setTolerance(1);
        balancePIDController.setTolerance(0.3, 1);
        balancePIDController.setSetpoint(0);

        configureMotors();

        // Dashboard settings
        setName("Drive");

        var commandList = Shuffleboard.getTab("Drive").getLayout("Commands", BuiltInLayouts.kList);
        commandList.add("Stop", stopCommand());
    }

    private void configureMotors() {
        frontLeftMotor.setNeutralMode(NeutralMode.Brake);
        frontRightMotor.setNeutralMode(NeutralMode.Brake);
        backLeftMotor.setNeutralMode(NeutralMode.Brake);
        backRightMotor.setNeutralMode(NeutralMode.Brake);

        frontLeftMotor.setSafetyEnabled(true);
        frontRightMotor.setSafetyEnabled(true);
        backLeftMotor.setSafetyEnabled(true);
        backRightMotor.setSafetyEnabled(true);

        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);

        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    // ACTIONS

    public void stop() {
        drive.stopMotor();
    }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        drive.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        drive.arcadeDrive(xSpeed, zRotation, squareInputs);
    }

    public void turnToTargetAngle() {
        final double angle = getGyroYaw();
        final double speed = turningPIDController.calculate(angle);
        drive.arcadeDrive(0, speed, false);
    }

    public void driveToTargetAngle() {
        final double angle = getGyroRoll();
        final double speed = balancePIDController.calculate(angle);
        drive.arcadeDrive(speed, 0, false);
    }

    public void driveToTargetDistance() {
        final double distance = (getDriveWheelPositions()[0] + getDriveWheelPositions()[1])/2;
        final double speed = drivePIDController.calculate(distance);
        drive.arcadeDrive(speed, 0, false);
    }

    public void setTargetTurningAngle(double setpoint) {
        turningPIDController.reset();
        turningPIDController.setSetpoint(setpoint);
    }

    public  void setTargetDrivingDistance(double setpoint) {
        drivePIDController.reset();
        drivePIDController.setSetpoint(setpoint);
    }

    // GETTERS

    /**
     * gets the gyro roll
     */
    public double getGyroRoll() {
        return gyro.getRoll();
    }

    /**
     * gets the gyro pitch
     */
    public double getGyroPitch() {
        return gyro.getPitch();
    }

    /**
     * gets the gyro yaw
     */
    public double getGyroYaw() {
        return gyro.getYaw();
    }

    public double[] getDriveWheelPositions() {
        return new double[]{
                frontLeftMotor.getSelectedSensorPosition(),
                frontRightMotor.getSelectedSensorPosition()};
    }

    // STATES

    /**
     * checks if you are stopped
     */
    public boolean stopped() {
        return Math.abs(frontLeftMotor.getSelectedSensorVelocity()) <= 0.1 &&
                Math.abs(frontRightMotor.getSelectedSensorVelocity()) <= 0.1;
    }

    /**
     * checks if you are at the wanted angle
     */
    public boolean atTurningAngle() {
        return turningPIDController.atSetpoint();
    }

    /**
     * checks if you are at the wanted distance
     */
    public boolean atDrivingDistance() {
        return drivePIDController.atSetpoint();
    }

    // COMMANDS

    /**
     * stop the drivetrain
     */
    public CommandBase stopCommand() {
        return this.runOnce(this::stop);
    }

    /**
     * makes command to drive with two speeds
     */
    public CommandBase driveWithSpeeds(double leftSpeed, double rightSpeed) {
        return this.run(() -> this.tankDrive(leftSpeed, rightSpeed, false));
    }

    /**
     * makes command to drive with joystick inputs
     */
    public CommandBase arcadeDriveWithJoysticks(DoubleSupplier xSpeed,
                                                DoubleSupplier zRotation,
                                                BooleanSupplier isNotSlowMode) {
        return this.run(() -> this.arcadeDrive(
                xSpeed.getAsDouble() * ((isNotSlowMode.getAsBoolean()) ? 1 : 0.5),
                zRotation.getAsDouble() * ((isNotSlowMode.getAsBoolean()) ? 0.55 : 0.27),
                true));
    }

    /**
     * makes command to turn to an angle
     */
    public CommandBase turnToAngle(double angle) {
        return this
                .runOnce(() -> this.setTargetTurningAngle(angle))
                .andThen(run(this::turnToTargetAngle).until(this::atTurningAngle));
    }

    public CommandBase balance() {
        return this.run(this::driveToTargetAngle).until(this.balancePIDController::atSetpoint);
    }

    public CommandBase driveToDistance(double distance) {
        return this
                .runOnce(() -> this.setTargetDrivingDistance(distance))
                .andThen(run(this::driveToTargetDistance)).until(this::atDrivingDistance);
    }
}
