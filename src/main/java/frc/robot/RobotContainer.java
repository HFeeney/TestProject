// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Logitech.Ports.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Devices
  WPI_TalonFX tfx0 = new WPI_TalonFX(0);
  WPI_TalonFX tfx1 = new WPI_TalonFX(1);
  WPI_TalonFX tfx2 = new WPI_TalonFX(2);
  WPI_TalonFX tfx3 = new WPI_TalonFX(3);

  DigitalInput proximitySensor = new DigitalInput(0);

  CANSparkMax frontLeftSwerve = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax frontRightSwerve = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax backLeftSwerve = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax backRightSwerve = new CANSparkMax(8, MotorType.kBrushless);
  
  // positive spark 2 is left lift down
  // positive spark 5 is right lift up
  CANSparkMax climb1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax climb2 = new CANSparkMax(5, MotorType.kBrushless);

  WPI_TalonSRX climbAngle = new WPI_TalonSRX(5);
  
  // positive spark 6 is flywheel one shooting out direction
  // positive spark 7 is flywheel two also shooting out direction
  CANSparkMax shooter1 = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(7, MotorType.kBrushless);
  
  // positive spark 1 is intaking (not outtaking)
  CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
  
  // positive delivery is reverse
  WPI_TalonSRX delivery = new WPI_TalonSRX(4);
  
  // intake solenoid reverse is extended, forward is retracted
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  DoubleSolenoid climbLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
  DoubleSolenoid hoodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
  DoubleSolenoid drivetrainShift = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
  AHRS navX = new AHRS(Port.kMXP);

  // Controller
  Logitech stick = new Logitech(0);
  JoystickButton a = new JoystickButton(stick, A);
  JoystickButton b = new JoystickButton(stick, B);
  JoystickButton x = new JoystickButton(stick, X);
  JoystickButton y = new JoystickButton(stick, Y);
  JoystickButton leftBumper = new JoystickButton(stick, LEFT_BUMPER);
  JoystickButton rightBumper = new JoystickButton(stick, RIGHT_BUMPER);

  Logitech stick2 = new Logitech(1);
  
  Command setMotorAxis;
  Command controlCommand;

  private static double multiplier = 0.2;
  private static final int angleLowLimit = -1488;
  private static final int angleHighLimit = -10;
  private static final double climb1LowLimit = -168;
  private static final double climb1HighLimit = -2;
  private static final double climb2LowLimit = -168;
  private static final double climb2HighLimit = -2;
  

  public RobotContainer() {
    climb1.setInverted(false);
    // climb2.follow(climb1, true)
    climb2.setInverted(true);

    a.whenPressed(new InstantCommand(() -> intakeSolenoid.set(Value.kForward)));
    b.whenPressed(new InstantCommand(() -> intakeSolenoid.set(Value.kReverse)));
    x.whenPressed(new InstantCommand(() -> hoodSolenoid.set(Value.kForward)));
    y.whenPressed(new InstantCommand(() -> hoodSolenoid.set(Value.kReverse))); // hood up
    leftBumper.whenPressed(new InstantCommand(() -> climbLock.set(Value.kForward)));
    rightBumper.whenPressed(new InstantCommand(() -> climbLock.set(Value.kReverse)));
    x.whenPressed(new InstantCommand(() -> {multiplier = 0.2;}));
    y.whenPressed(new InstantCommand(() -> {multiplier = 0.6;}));
    
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter1.setInverted(true);
    shooter1.setOpenLoopRampRate(2);
    shooter2.setOpenLoopRampRate(2);

    controlCommand = new RunCommand(
      () -> {
        // double shooterSpeed = stick.getRawAxis(0);

        // SmartDashboard.putNumber("shooter speed", shooterSpeed);
        // shooter1.set(stick.getRawAxis(0));
        // shooter2.set(stick.getRawAxis(0));

        // SmartDashboard.putNumber("shooter rpm", shooter1.getEncoder().getVelocity());
        
        // delivery.set(stick.getRawAxis(4));

        // intakeMotor.set(stick.getRawAxis(3));

        // CLIMB
        double angleInput = stick.getRawAxis(RIGHT_STICK_X) * multiplier;
        int anglePosition = climbAngle.getSensorCollection().getQuadraturePosition();

        if (climbLock.get() == Value.kReverse)
          angleInput = 0.0;

        if (anglePosition <= angleLowLimit && angleInput < 0)
          climbAngle.set(angleInput);
        else if (anglePosition >= angleHighLimit && angleInput > 0)
          climbAngle.set(angleInput);
        else if (anglePosition >= angleLowLimit && anglePosition <= angleHighLimit)
          climbAngle.set(angleInput);
        else {
          climbAngle.set(0.0);
        }
        
        double extendInput = stick.getRawAxis(LEFT_STICK_Y) * 0.5;
        double extendPosition1 = climb1.getEncoder().getPosition();
        double extendPosition2 = climb2.getEncoder().getPosition();

        if (extendPosition1 <= climb1LowLimit && extendInput > 0)
          climb1.set(extendInput);
        else if (extendPosition1 >= climb1HighLimit && extendInput < 0)
          climb1.set(extendInput);
        else if (extendPosition1 >= climb1LowLimit && extendPosition1 <= climb1HighLimit)
          climb1.set(extendInput);
        else
          climb1.set(0.0);

        if (extendPosition2 <= climb2LowLimit && extendInput > 0)
          climb2.set(extendInput);
        else if (extendPosition2 >= climb2HighLimit && extendInput < 0)
          climb2.set(extendInput);
        else if (extendPosition2 >= climb2LowLimit && extendPosition2 <= climb2HighLimit)
          climb2.set(extendInput);
        else
          climb2.set(0.0);

        SmartDashboard.putNumber("extend input", extendInput);
        SmartDashboard.putNumber("angle input", angleInput);
        SmartDashboard.putNumber("climb1 encoder", climb1.getEncoder().getPosition());
        SmartDashboard.putNumber("climb2 encoder", climb2.getEncoder().getPosition());
        SmartDashboard.putNumber("climb angle encoder", climbAngle.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("current multiplier", multiplier);
        if (stick2.getRawAxis(RIGHT_TRIGGER) > 0.5) {
          climb1.set(stick2.getRawAxis(LEFT_STICK_Y));
          climb2.set(stick2.getRawAxis(RIGHT_STICK_Y));
        }
      }
    );  

  }


}
