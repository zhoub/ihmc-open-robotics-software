package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@Tag("fast")
public class MessagerAStarDataSetTest extends MessagerPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
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
      MessagerAStarDataSetTest test = new MessagerAStarDataSetTest();
      String prefix = "unitTestDataSets/test/";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_204917_FlatGround");
      test.tearDown();

   }
}
