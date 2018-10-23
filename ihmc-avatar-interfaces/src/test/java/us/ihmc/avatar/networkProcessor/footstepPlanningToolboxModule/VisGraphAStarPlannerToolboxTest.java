package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pubsub.DomainFactory;

@Tag("fast")
public class VisGraphAStarPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   @Override
   @Test(timeout = 500000)
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

   @Override
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
      VisGraphAStarPlannerToolboxTest test = new VisGraphAStarPlannerToolboxTest();
      String prefix = "unitTestDataSets/test/";
      VISUALIZE = true;
      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_205120_BodyPathPlannerEnvironment");
      PrintTools.info("Test passed.");
//      test.tearDown();
   }
}
