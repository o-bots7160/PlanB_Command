package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

public class TimedDoubleSolenoid implements Sendable
{
   private DoubleSolenoid _solenoid;
   private DoubleSolenoidSim _solenoidSim;
   private OnOffDelay     delay;
   //
   //   Create TimedDoubleSolenoid with pins and single extend/retract time
   //
   //
   public TimedDoubleSolenoid( int retract, int extend, double new_extend_time )
   {
      _solenoid    = new DoubleSolenoid( 9, PneumaticsModuleType.REVPH, retract, extend );
      _solenoidSim = new DoubleSolenoidSim(  9, PneumaticsModuleType.REVPH, retract, extend );
      delay        = new OnOffDelay( new_extend_time, new_extend_time, ()->{ return _solenoid.get() == Value.kForward; } );
      _solenoid.set( Value.kReverse );
   }
   //
   //   Create TimedDoubleSolenoid with pins and extend and retract times
   //
   //
   public TimedDoubleSolenoid( int retract, int extend, double new_extend_time, double new_retract_time )
   {
      _solenoid    = new DoubleSolenoid( 9, PneumaticsModuleType.REVPH, retract, extend );
      _solenoidSim = new DoubleSolenoidSim(  9, PneumaticsModuleType.REVPH, retract, extend );
      delay        = new OnOffDelay( new_extend_time, new_retract_time, ()->{ return _solenoid.get() == Value.kForward; } );
      _solenoid.set( Value.kReverse );
      _solenoidSim.set( Value.kReverse );
   }
   //
   //   Extend cylinder if not already extended/extending
   //
   //
   public void extend( )
   {
      _solenoid.set( Value.kForward );
      _solenoidSim.set( Value.kForward );
      delay.get( );  // Catch changing state
   }
   //
   //   Retract cylinder if not already retracted/retracting
   //
   //
   public void retract( )
   {
      _solenoid.set( Value.kReverse );
      _solenoidSim.set( Value.kReverse );
      delay.get( );  // Catch changing state
   }
   //
   //   Return true if still extending
   //
   //
   public boolean isExtending( )
   {
      return ( _solenoid.get( ) == Value.kForward ) && !isExtended( );
   }
   //
   //   Return true if completed extending
   //
   //
   public boolean isExtended( )
   {
      return delay.get( );
   }
   //
   //   Return true if still retracting
   //
   //
   public boolean isRetracting( )
   {
      return ( _solenoid.get( ) == Value.kReverse ) && ! isRetracted( );
   }
   //
   //   Return true if completed retracting
   //
   //
   public boolean isRetracted( )
   {
      return !delay.get( );
   }
   //
   //   Make this a sendable class.
   //
   //
   @Override
   public void initSendable( SendableBuilder builder )
   {
      // Publish the solenoid state to telemetry.
      builder.addBooleanProperty("raw", ()->{ return _solenoid.get() == Value.kForward; }, null);
      builder.addBooleanProperty("filtered", ()->{ return delay.get(); }, null);
   }
}