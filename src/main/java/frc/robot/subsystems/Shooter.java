package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.Joystick;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase
{
   //
   //  Target positions for shooter when not using calculated position
   //
   //
   final double stow_angle   = Math.toRadians(167.0);
   final double intake_angle = Math.toRadians(224.0);
   final double amp_angle    = Math.toRadians(208.0);
   //
   //  Stateless flags for each setting
   //
   //
   private boolean calculating    = false;      // determing angle and speed from distance
   private boolean shooting       = false;      // shooting into amp or speaker
   private double  angle_target   = stow_angle; // position going to motor controller
   private double  intake_target  = 0.0;
   private double  shooter_target = 0.0;
   private double  last_distance  = 0.0;
   BooleanSupplier trigger;
   DoubleSupplier  distance;

   private TimeOfFlight sensor = new TimeOfFlight( 0 );

   private CANSparkMax      _angle_motor;
   private PIDController    pid_angle     = new PIDController( 11.000, 0.01, 0.1 );
   private DutyCycleEncoder angle_encoder = new DutyCycleEncoder( 0 );
   private double           angle_volts;
   private double           angle_current;

   private TalonFX          _intake;

   private TalonFX _topShoot;
   private Slot0Configs pid_shoot = new Slot0Configs();
   //private PIDController pid_topShoot;
   //private RelativeEncoder en_topShoot;

   private TalonFX _bottomShoot;
   //
   //  Simulation objects
   //
   //
   private DutyCycleEncoderSim angle_encoder_sim = new DutyCycleEncoderSim( angle_encoder);
   Joystick Joystick = new Joystick( 1 );
   private  TalonFXSimState simTopshoot;
   private  SlewRateLimiter angle_limiter = new SlewRateLimiter( 1.5 );

   public Shooter( BooleanSupplier new_trigger, DoubleSupplier new_distance )
   {
      trigger   = new_trigger;
      distance  = new_distance;

      sensor.setRangingMode(RangingMode.Short, 24);

      _angle_motor = new CANSparkMax(56, MotorType.kBrushless);
      _angle_motor.setSmartCurrentLimit(40);
      _angle_motor.setInverted(     false);
      _angle_motor.enableSoftLimit( SoftLimitDirection.kReverse, true );
      _angle_motor.setSoftLimit(    SoftLimitDirection.kReverse, 0 );        //lower limit //FIXME
      _angle_motor.enableSoftLimit( SoftLimitDirection.kForward, true );
      _angle_motor.setSoftLimit(    SoftLimitDirection.kForward, 100 );      //upper limit //FIXME
      _angle_motor.setIdleMode(     IdleMode.kBrake );

      pid_angle.enableContinuousInput( 0.0, 2.0 * Math.PI );
      pid_angle.setTolerance( Math.toRadians( 1.0 ) );
      pid_angle.setIZone(     Math.toRadians( 2.0 ) );
      //pid_angle.setFF         ( 0.0 );
      //pid_angle.setOutputRange( -0.9, 0.9 );

      _intake = new TalonFX(53);
      //_intake.setControl( );
      // _intake.setSmartCurrentLimit(35);
      // _intake.setInverted(true);
      // _intake.enableSoftLimit(SoftLimitDirection.kReverse, true);
      // _intake.setSoftLimit(SoftLimitDirection.kReverse, 6);        //lower limit //FIXME
      // _intake.enableSoftLimit(SoftLimitDirection.kForward, true);
      // _intake.setSoftLimit(SoftLimitDirection.kForward, 170);      //upper limit //FIXME
      _intake.setNeutralMode(NeutralModeValue.Brake);
      // pid_intake = _intake.getPIDController();
      // pid_intake.setP          ( 0.8 );
      // pid_intake.setI          ( 0.0 );
      // pid_intake.setD          ( 0.0 );
      // pid_intake.setIZone      ( 0.0 );
      // pid_intake.setFF         ( 0.0 );
      // pid_intake.setOutputRange( -0.9, 0.9 );
      // en_intake = _intake.getEncoder();
      // Do we need this? _intake.burnFlash();

      _topShoot = new TalonFX(54);
      pid_shoot.kS = 0.5;
      pid_shoot.kV = 0.0;
      pid_shoot.kA = 0.0;
      pid_shoot.kP = 0.5;
      pid_shoot.kI = 0.0;
      pid_shoot.kD = 0.0;
      _topShoot.getConfigurator().apply(pid_shoot);
      // _topShoot.setVoltage(35);
      // _topShoot.setInverted(true);
      // _topShoot.setReverseLimit(SoftLimitDirection.kReverse, true);
      // _topShoot.setSoftLimit(SoftLimitDirection.kReverse, 6);        //lower limit //FIXME
      // _topShoot.enableSoftLimit(SoftLimitDirection.kForward, true);
      // _topShoot.setSoftLimit(SoftLimitDirection.kForward, 170);      //upper limit //FIXME
      // _topShoot.setNeutralMode(NeutralModeValue.Brake);
      // pid_topShoot = _topShoot.
      // pid_topShoot.setP          ( 0.8 );
      // pid_topShoot.setI          ( 0.0 );
      // pid_topShoot.setD          ( 0.0 );
      // pid_topShoot.setIZone      ( 0.0 );
      // pid_topShoot.setFF         ( 0.0 );
      // pid_topShoot.setOutputRange( -0.9, 0.9 );
      // en_topShoot = _topShoot.getEncoder();
      // Do we need this? _topShoot.burnFlash();

      _bottomShoot = new TalonFX(55);
      _bottomShoot.setControl(new Follower(54, true));
      if ( Robot.isSimulation() )
      {
         simTopshoot = new TalonFXSimState( _topShoot );
      }
   }
   public void updateTrigger( BooleanSupplier new_trigger )
   {
      trigger = new_trigger;
   }
   public void updateDistance( DoubleSupplier new_distance )
   {
      distance = new_distance;
   }
   //
   //   Prep shooter for stowing
   //
   //
   public Command stow( )
   {
      return new PrintCommand( "Shooter: stow" ).andThen (
             new FunctionalCommand(
                 ()->{ calculating    = false;
                       shooting       = false;
                       angle_target   = stow_angle;
                       intake_target  = 0.0;
                       shooter_target = 0.0;
                       pid_angle.reset( ); },
                 ()->{ },
                 interrupted ->{ },
                 ()->{ return isReady( ); },
                 this ) );
   }
   //
   //   Prep shooter for ejecting note
   //
   //
   public Command eject( )
   {
      return new FunctionalCommand(
         ()->{ calculating    = false;
               shooting       = false;
               angle_target   = stow_angle;
               intake_target  = -40.0;
               shooter_target =   0.0;
               pid_angle.reset( ); },
         ()->{ },
         interrupted ->{ },
         ()->{ return isReady( ) && ! haveNote(); },
         this);
   }
   //
   //   Prep shooter for intaking Note
   //
   //
   public Command intake( )
   {
      return new PrintCommand( "Shooter: intake" ).andThen (
             new FunctionalCommand(
                ()->{ calculating    = false;
                      shooting       = false;
                      angle_target   = intake_angle;
                      intake_target  =  40.0;
                      shooter_target =   0.0;
                      pid_angle.reset( ); },
                ()->{ },
                interrupted ->{ },
                ()->{ return isReady( ) && haveNote(); },
                this ) );
   }
   //
   //   Prep shooter for shooting Note at speaker
   //
   //
   public Command speaker( )
   {
      return new PrintCommand( "Shooter: speaker" ).andThen (
             new FunctionalCommand(
                ()->{ calculating    = true;
                      shooting       = true;
                      angle_target   = calculateAngle( );
                      intake_target  =  80.0;
                      shooter_target = calculateRPM( );
                      pid_angle.reset( ); },
                ()->{ },
                interrupted ->{ },
                ()->{ return isReady( ) && ! haveNote(); },
                this ) );
   }
   //
   //   Prep shooter for shooting Note at amp
   //
   //
   public Command shootAmp( )
   {
      return new PrintCommand( "Shooter: amp" ).andThen (
             new FunctionalCommand(
                ()->{ calculating    = false;
                      shooting       = true;
                      angle_target   = amp_angle;
                      intake_target  =  80.0;
                      shooter_target =   0.5; },
                ()->{ },
                interrupted ->{ },
                ()->{ return isReady( ) && !haveNote(); },
                this ) );
   }
   //
   //   Return true if shooter at angle and speeds
   //
   //
   public boolean isReady( )
   {
      boolean return_value = true;
      double  velocity_error;
      //
      //   Is Angle at position
      //
      //
      return_value &= pid_angle.atSetpoint( );
      //
      //   Is shooter at velocity position
      //
      //
      if ( shooter_target > 1.0 )
      {
         velocity_error = Math.abs(_topShoot.getVelocity().getValueAsDouble() - shooter_target );
         if ( velocity_error > 50.0 )  // TODO: what should this be?
         {
            //return_value = false;  // TODO: how do this?
         }
      }
      return return_value;
   }
   //
   //   Return true if note detected in shooter
   //
   //
   public boolean haveNote( )
   {
      boolean return_value = false;
      double distance      = sensor.getRange();

      if ( Robot.isReal() )
      {
         if ( sensor.isRangeValid( ) )
         {
            last_distance = distance;
         }
         else
         {
            distance = last_distance;
         }
      }
      else
      {
         //
         //  Simulation sets this from keypress
         //
         //
         distance = last_distance;
      }
   
      if ( distance < 215.0 )
      {
         return_value = true;
      }
      return ( return_value );
   }
      // double angleRadians = angle_encoder.getAbsolutePosition() * 2.0 * Math.PI;
      // double armVolts     = Math.cos( angle_target - travel_angle ) * -0.00;
      // SmartDashboard.putNumber("shooter/angle position", Math.toDegrees( angleRadians ));
      // SmartDashboard.putNumber("shooter/angle target", Math.toDegrees( angle_target ));
      // SmartDashboard.putNumber("shooter/TOF", sensor.getRange() );
      // SmartDashboard.putBoolean("shooter/manual", manual.getAsBoolean() );
      // SmartDashboard.putNumber("shooter/encoder", angle_encoder.getPosition() );
      // SmartDashboard.putNumber("shooter/armVolts", armVolts );
   //
   //   Calculate the target angle for the distance from the speaker/amp
   //
   //
   private double calculateAngle( )
   {
      double distance     = this.distance.getAsDouble();
      double min_distance = 0.83;
      double max_distance = 4.67;
      double max_angle    = Math.toRadians(27); //22
      double new_target;

      if (distance < min_distance)
      {
         distance = min_distance;
      }
      else if (distance > max_distance)
      {
         distance = max_distance;
      }

      new_target = stow_angle + ((max_angle)*((max_distance-distance)/(max_distance-min_distance))) - Math.toRadians(9.0);
      return new_target;
   }
   //
   //   Calculate the target RPM for the distance from the speaker/amp
   //
   //
   private double calculateRPM( )
   {
      return 4150; // FIXME currently set to 65% of TalonFX max 6380;
   }
   //
   //   Simulate shooter movement for testing
   //
   //
   @Override
   public void periodic()
   {
      // should equal  V to hold at 0 radians * Math.cos( target angle radians )
      double  armVolts = 0.0; // for gravity compensation

      if ( calculating )
      {
         angle_target   = calculateAngle( );
         shooter_target = calculateRPM( );
      }
      angle_current = angle_encoder.getAbsolutePosition( ) * 2.0 * Math.PI;
      pid_angle.setSetpoint( angle_target );
      angle_volts = pid_angle.calculate( angle_current ) + armVolts;
      _angle_motor.setVoltage( angle_volts );

      _topShoot.setControl( new VelocityVoltage( shooter_target ) );
      //
      // Publish the shooter telemetry
      //
      //
      SmartDashboard.putData("Shooter", this );
   }
   //
   //   Simulate shooter movement for testing
   //
   //
   public void simulationPeriodic()
   {
      //System.out.println( "sim:" + Math.toDegrees( angle_current ) + " " + angle_volts + " " + pid_angle.getPositionError() );
      angle_current = angle_limiter.calculate( angle_current + angle_volts * 0.01 );
      if ( angle_current < stow_angle )
      {
         angle_current = stow_angle;
      }
      else if ( angle_current > intake_angle )
      {
         angle_current = intake_angle;
      }
      //System.out.println( "sim:" + Math.toDegrees( angle_current ) + " " + angle_volts + " " + pid_angle.getPositionError() );
      angle_encoder_sim.setAbsolutePosition( angle_current / 2.0 / Math.PI );

      if ( Joystick.getRawButton( 1 ) )
      {
         last_distance = 200;
      }
      else
      {
         last_distance = 400;
      }
      simTopshoot.setRotorVelocity( shooter_target );
      //
      //   Should intake be spinning
      //
      //
      if ( !shooting || ( trigger.getAsBoolean( ) && isReady() ) )
      {
         _intake.setControl( new DutyCycleOut( intake_target ) );
      }
      else
      {
         _intake.setControl( new DutyCycleOut( 0.0 ) );
      }
   }
   //
   // Create a SysIdRoutine for a Talon motor
   //
   //
   public static SysIdRoutine setMotorSysIdRoutine( Config config, TalonFX motor, SubsystemBase subsystem )
   {
      return new SysIdRoutine(config, new Mechanism(
         (Measure<Voltage> voltage) -> { motor.setControl( new VoltageOut( voltage.in( Volts ) ) ); },
          log -> { log.motor( "topShoot" )
                  .voltage( mutable( Volts.of( motor.getMotorVoltage().getValue() ) ) )
                  .linearVelocity( mutable( MetersPerSecond.of( motor.getVelocity().getValue() ) ) ); },
          subsystem ) );
   }
   //
   // Create a SysIdRoutine for a Talon motor
   //
   //
   public static SysIdRoutine setMotorSysIdRoutine( Config config, CANSparkMax motor, SubsystemBase subsystem )
   {
      return new SysIdRoutine(config, new Mechanism(
         (Measure<Voltage> voltage) -> { motor.setVoltage( voltage.in( Volts ) ); },
          log -> { log.motor( "topShoot" )
                  .voltage( mutable( Volts.of( motor.getBusVoltage() * motor.getAppliedOutput() ) ) )
                  .linearVelocity( mutable( MetersPerSecond.of( motor.getEncoder().getVelocity() ) ) ); },
          subsystem ) );
   }
   //
   // Create a SysIdRoutine Command for the angle motor
   //
   //
   public Command setAngleSysIdTest( )
   {
      return new PrintCommand( "Angle SysId Start" ).andThen(
             setMotorSysIdRoutine( new Config(), _angle_motor, this ).quasistatic( Direction.kForward ) ).andThen( 
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _angle_motor, this ).quasistatic( Direction.kReverse ) ).andThen(   
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _angle_motor, this ).dynamic( Direction.kForward ) ).andThen( 
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _angle_motor, this ).dynamic( Direction.kReverse ) ).andThen(
             new PrintCommand( "Angle SysId test complete" ) );
   }
   //
   // Create a SysIdRoutine Command for the top shooter motor
   //
   //
   public Command setTopSysIdTest( )
   {
      return setMotorSysIdRoutine( new Config(), _topShoot, this ).quasistatic( Direction.kForward ).andThen( 
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _topShoot, this ).quasistatic( Direction.kReverse ).andThen(   
             new WaitCommand( 10.0 ) ) ).andThen(
             setMotorSysIdRoutine( new Config(), _topShoot, this ).dynamic( Direction.kForward ).andThen( 
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _topShoot, this ).dynamic( Direction.kReverse ) ) );
   }
   //
   // Create a SysIdRoutine Command for the bottom shooter motor
   //
   //
   public Command setBotSysIdTest( )
   {
      return setMotorSysIdRoutine( new Config(), _bottomShoot, this ).quasistatic( Direction.kForward ).andThen( 
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _bottomShoot, this ).quasistatic( Direction.kReverse ).andThen(   
             new WaitCommand( 10.0 ) ) ).andThen(
             setMotorSysIdRoutine( new Config(), _bottomShoot, this ).dynamic( Direction.kForward ).andThen( 
             new WaitCommand( 10.0 ) ).andThen(
             setMotorSysIdRoutine( new Config(), _bottomShoot, this ).dynamic( Direction.kReverse ) ) );
   }
   //
   //   Make this a sendable class.
   //
   //
   @Override
   public void initSendable( SendableBuilder builder )
   {
      super.initSendable( builder );
      builder.setSmartDashboardType( "what" );
      builder.addDoubleProperty("Angle Target", ()->{ return Math.toDegrees( angle_target  ); }, null );
      builder.addDoubleProperty("Angle Pose",   ()->{ return Math.toDegrees( angle_current ); }, null );
   }
}