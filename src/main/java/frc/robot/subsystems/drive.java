package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveTrainConstants;

public class drive extends SubsystemBase {

    WPI_TalonFX frontLeft = new WPI_TalonFX(driveTrainConstants.frontLeftPort);
    WPI_TalonFX frontRight = new WPI_TalonFX(driveTrainConstants.frontRightPort);
    WPI_TalonFX backRight = new WPI_TalonFX(driveTrainConstants.backRightPort);
    WPI_TalonFX backLeft = new WPI_TalonFX(driveTrainConstants.backLeftPort);

    DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

    public drive() {

    }

    private void configureMotors() {
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
