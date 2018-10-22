package us.ihmc.footstepPlanning.roughTerrainPlanning;

import us.ihmc.footstepPlanning.FootstepPlannerType;

public class MessagerAStarOnRoughTerrainTest extends MessagerFootstepPlannerOnRoughTerrainTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

}
