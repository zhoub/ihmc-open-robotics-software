package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@Tag("slow")
public class MessagerSimpleBodyPathDataSetTest extends MessagerPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.SIMPLE_BODY_PATH;
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
      MessagerSimpleBodyPathDataSetTest test = new MessagerSimpleBodyPathDataSetTest();
      String prefix = "unitTestData/testable/";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171218_204953_FlatGroundWithWall");
      test.tearDown();

   }
}
