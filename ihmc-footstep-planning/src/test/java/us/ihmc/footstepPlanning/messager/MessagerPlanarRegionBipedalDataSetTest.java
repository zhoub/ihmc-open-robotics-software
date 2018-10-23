package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.PrintTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@Tag("slow")
public class MessagerPlanarRegionBipedalDataSetTest extends MessagerPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLANAR_REGION_BIPEDAL;
   }

   @Override
   @Test // timeout = 500000
   public void testDatasetsWithoutOcclusion()
   {
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   @Override
   @Test // timeout = 500000
   @Tag("in-development")
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(dataset -> runAssertions(dataset));
   }

   public static void main(String[] args) throws Exception
   {
//      VISUALIZE = true;
      MessagerPlanarRegionBipedalDataSetTest test = new MessagerPlanarRegionBipedalDataSetTest();
      String prefix = "unitTestDataSets/test/";
      test.setup();
      PrintTools.info("Running test.");
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171215_214801_StairsUpDown");
      PrintTools.info("Test finished.");
      test.tearDown();

   }
}
