package team6829.common;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import team6829.common.transforms.ITransform;



public class DriveTrain extends Subsystem {

	private Command defaultCommand;

	private WPI_TalonSRX leftMaster;
	private WPI_VictorSPX leftFollower;
	private WPI_TalonSRX rightMaster;
	private WPI_VictorSPX rightFollower;

	private AHRS navX;
	
	private int leftMasterCanId;
	private int leftFollowerCanId;
	private int rightMasterCanId;
	private int rightFollowerCanId;


	
	/**
	 * Returns an Image object that can then be painted on the screen. 
	 * The url argument must specify an absolute {@link URL}. The name
	 * argument is a specifier that is relative to the url argument. 
	 * <p>
	 * This method always returns immediately, whether or not the 
	 * image exists. When this applet attempts to draw the image on
	 * the screen, the data will be loaded. The graphics primitives 
	 * that draw the image will incrementally paint on the screen. 
	 *
	 * @param  leftMasterCanId  an absolute URL giving the base location of the image
	 * @param  leftFollowerCanId the location of the image, relative to the url argument
	 * @param  rightMasterCanId the location of the image, relative to the url argument
	 * @param  rightFollowerCanId  an absolute URL giving the base location of the image
	 
	* @return      the image at the specified URL
	*/

	public DriveTrain(int leftMasterCanId, int leftFollowerCanId, int rightMasterCanId, int rightFollowerCanId) {

		this.leftMasterCanId = leftMasterCanId;
		this.leftFollowerCanId = leftFollowerCanId;
		this.rightMasterCanId = rightMasterCanId;
		this.rightFollowerCanId = rightFollowerCanId;
		
		defaultDirection();
		defaultLeftRight(); //Talons instantiated here
		
		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);

		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20, 10);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20, 10);

		navX = new AHRS(SPI.Port.kMXP);

	}

	/**
	 * 
	 */
	public void setCommandDefault(Command defaultCommand) {
		this.defaultCommand = defaultCommand;
		initDefaultCommand();
	}

	public void initDefaultCommand() {
		setDefaultCommand(this.defaultCommand);
	}

	public void arcadeDrive(double throttlePower, double turnPower, double deadband, ITransform transform) {
		double leftMotorOutput;
		double rightMotorOutput;

		throttlePower = transform.transform(throttlePower);
		turnPower = transform.transform(turnPower);
	
		double maxInput = Math.copySign(Math.max(Math.abs(throttlePower), Math.abs(turnPower)), throttlePower);

		if (throttlePower >= 0.0) {
			// First quadrant, else second quadrant
			if (turnPower >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttlePower - turnPower;
			} else {
				leftMotorOutput = throttlePower + turnPower;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (turnPower >= 0.0) {
				leftMotorOutput = throttlePower + turnPower;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttlePower - turnPower;
			}
		}

		throttlePower = limit(throttlePower);
		throttlePower = applyDeadband(throttlePower, deadband); // set throttle deadband here

		turnPower = limit(turnPower);
		turnPower = applyDeadband(turnPower, deadband); // set rotate deadband here
		
		leftMaster.set(ControlMode.PercentOutput, leftMotorOutput);
		rightMaster.set(ControlMode.PercentOutput, rightMotorOutput);

	}

	public void reverseDirection() {
		leftMaster.setInverted(false);
		leftFollower.setInverted(false);
		rightMaster.setInverted(true);
		rightFollower.setInverted(true);
	}
	
	public void defaultDirection() {
		leftMaster.setInverted(true);
		leftFollower.setInverted(true);
		rightMaster.setInverted(false);
		rightFollower.setInverted(false);
	}
	
	public void defaultLeftRight() {
		leftMaster = new WPI_TalonSRX(leftMasterCanId);
		leftFollower = new WPI_VictorSPX(leftFollowerCanId);
		rightMaster = new WPI_TalonSRX(rightMasterCanId);
		rightFollower = new WPI_VictorSPX(rightFollowerCanId);
		
	}

	public void reverseLeftRight() {
		rightMaster = new WPI_TalonSRX(leftMasterCanId);
		rightFollower = new WPI_VictorSPX(leftFollowerCanId);
		leftMaster = new WPI_TalonSRX(rightMasterCanId);
		leftFollower = new WPI_VictorSPX(rightFollowerCanId);
	}

	public void setLeftDrivePower(double power) {
		leftMaster.set(ControlMode.PercentOutput, limit(power));
	}

	public void setRightDrivePower(double power) {
		rightMaster.set(ControlMode.PercentOutput, limit(power));
	}

	public void setLeftRightDrivePower(double leftPower, double rightPower) {
		setLeftDrivePower(leftPower);
		setRightDrivePower(rightPower);
	}
	
	public int getLeftEncoderPosition() {
		return leftMaster.getSensorCollection().getQuadraturePosition();
	}
	
	public int getRightEncoderPosition() {
		return rightMaster.getSensorCollection().getQuadraturePosition();
	}

	public int getLeftEncoderVelocity() {
		return leftMaster.getSensorCollection().getQuadratureVelocity();
	}

	public int getRightEncoderVelocity() {
		return rightMaster.getSensorCollection().getQuadratureVelocity();
	}

	public double getLeftVoltage() {
		return leftMaster.getMotorOutputVoltage();
	}

	public double getRightVoltage() {
		return rightMaster.getMotorOutputVoltage();
	}

	public double getLeftPercentOutput() {
		return leftMaster.getMotorOutputPercent();
	}
	
	public double getRightPercentOutput() {
		return rightMaster.getMotorOutputPercent();
	}
	
	public void stop() {

		leftMaster.stopMotor();
		rightMaster.stopMotor();

	}

	public TalonSRX getRightMaster() {
		return rightMaster;
	}

	public TalonSRX getLeftMaster() {
		return leftMaster;
	}

	public void zeroEncoders() {
		rightMaster.getSensorCollection().setQuadraturePosition(0, 10);
		leftMaster.getSensorCollection().setQuadraturePosition(0, 10);

	}

	public double getAngle() {
		return navX.getAngle();
	}

	public void zeroAngle() {
		navX.zeroYaw();
	}

	public boolean isCalibrating() {
		return navX.isCalibrating();
	}

	public boolean isConnected() {
		return navX.isConnected();
	}

	private double limit(double value) {
		if (value > 1.0) {
			return 1.0;
		}
		if (value < -1.0) {
			return -1.0;
		}
		return value;
	}

	private double applyDeadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

}