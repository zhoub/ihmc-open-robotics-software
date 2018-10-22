package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

@Tag("slow")
public class RemoteStandaloneSimpleBodyPathDataSetTest extends RemoteStandalonePlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.SIMPLE_BODY_PATH;
   }

   @Override
   @Test
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 37.6)
   public void testDatasetsWithoutOcclusion()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   @Override
   @Test
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 125.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(dataset -> runAssertions(dataset));
   }

   @Test
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 37.6, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionRTPS()
   {
      pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      setup();
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   public static void main(String[] args) throws Exception
   {
      RemoteStandaloneSimpleBodyPathDataSetTest test = new RemoteStandaloneSimpleBodyPathDataSetTest();
      String prefix = "unitTestData/testable/";
      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_204953_FlatGroundWithWall");
      test.tearDown();

   }
}
