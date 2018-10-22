package us.ihmc.tools.inputDevices.keyboard.linux;

import org.junit.jupiter.api.Test;

public class RepeatingReleasedEventsFixerTest
{
   @Test
   public void testInstallAndRemove()
   {
      RepeatingReleasedEventsFixer repeatingReleasedEventsFixer = new RepeatingReleasedEventsFixer();
      repeatingReleasedEventsFixer.install();
      
      // TODO test some dispatched events here
      
      repeatingReleasedEventsFixer.remove();
   }
}
