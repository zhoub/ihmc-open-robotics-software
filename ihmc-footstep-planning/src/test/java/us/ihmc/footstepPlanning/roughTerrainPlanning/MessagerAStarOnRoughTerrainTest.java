package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@Tag("fast")
public class MessagerAStarOnRoughTerrainTest extends MessagerFootstepPlannerOnRoughTerrainTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

}
