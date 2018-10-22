package us.ihmc.humanoidRobotics.kryo;

import org.junit.jupiter.api.Test;
import us.ihmc.communication.kryo.KryoNetClassListTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class IHMCCommunicationKryoNetClassListTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testAllClassesRegisteredArePackets()
   {
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      KryoNetClassListTestHelper.testAllClassesRegisteredArePackets(netClassList);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.5)
   @Test
   public void testAllPacketFieldsAreRegistered()
         throws InstantiationException, IllegalAccessException, NoSuchFieldException, SecurityException, IllegalArgumentException
   {
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      KryoNetClassListTestHelper.testAllPacketFieldsAreRegistered(netClassList);
   }
}
