package us.ihmc.valkyrie;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class ValkyrieBranchJobBalancerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 180.0)
   @Test(timeout = 30000)
   public void testExtraTimeToSyncJobs()
   {

   }
}
