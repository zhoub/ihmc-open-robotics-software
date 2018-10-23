package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

@Tag("slow")
public class RemoteStandalonePlanarRegionBipedalDataSetTest extends RemoteStandalonePlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLANAR_REGION_BIPEDAL;
   }

   @Override
   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 93.2)
   public void testDatasetsWithoutOcclusion()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   @Override
   @Test(timeout = 500000)
   @Tag("in-development")
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(dataset -> runAssertions(dataset));
   }

   @Test(timeout = 500000)
   @Tag("in-development")
   public void testDatasetsWithoutOcclusionRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   public static void main(String[] args) throws Exception
   {
      VISUALIZE = true;
      RemoteStandalonePlanarRegionBipedalDataSetTest test = new RemoteStandalonePlanarRegionBipedalDataSetTest();
      String prefix = "unitTestData/testable/";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_205120_BodyPathPlannerEnvironment");
      test.tearDown();

   }
}
