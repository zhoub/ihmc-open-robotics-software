package us.ihmc.footstepPlanning.roughTerrainPlanning;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@Tag("fast")
@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class MessagerAStarOnRoughTerrainTest extends MessagerFootstepPlannerOnRoughTerrainTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

}
