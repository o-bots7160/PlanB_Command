package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

public class OnOffDelay
{
   private final Timer           timer     = new Timer();
   private       boolean         last_raw  = false;
   private       boolean         filtered  = false;
   private       double          on_delay;
   private       double          off_delay;
   private       BooleanSupplier rawValue;

   public OnOffDelay( double new_on_delay, double new_off_delay, BooleanSupplier new_rawValue )
   {
      on_delay  = new_on_delay;
      off_delay = new_off_delay;
      rawValue  = new_rawValue;
   }

   public void setOnOffDelay( double new_on_delay, double new_off_delay )
   {
      on_delay  = new_on_delay;
      off_delay = new_off_delay;
   }
   //
   //  Using timer deterimine if a signal is supposed to be on or off
   //  after applying the appropriate delays
   //
   //
   public boolean get( )
   {
      boolean current_raw = rawValue.getAsBoolean();

      if ( last_raw )
      {
         if ( current_raw )
         {
            if ( timer.hasElapsed( on_delay ) )
            {
               filtered = true;
               timer.stop();
            }
         }
         else
         {
            timer.restart();
         }
      }
      else
      {
         if ( ! current_raw )
         {
            if( timer.hasElapsed( off_delay ) )
            {
               filtered = false;
               timer.stop();
            }
         }
         else
         {
            timer.restart();
         }
      }
      last_raw = current_raw;
      return filtered;
   }
}