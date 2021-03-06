package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class FootstepCostBuilder
{
   private final RequiredFactoryField<FootstepPlannerParameters> footstepPlannerParameters = new RequiredFactoryField<>("footstepPlannerParameters");
   private final RequiredFactoryField<FootstepNodeSnapperReadOnly> snapper = new RequiredFactoryField<>("snapper");

   private final OptionalFactoryField<Boolean> includeHeightCost = new OptionalFactoryField<>("includeHeightCost");
   private final OptionalFactoryField<Boolean> includePitchAndRollCost = new OptionalFactoryField<>("includePitchAndRollCost");

   public void setFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.footstepPlannerParameters.set(footstepPlannerParameters);
   }

   public void setSnapper(FootstepNodeSnapperReadOnly snapper)
   {
      this.snapper.set(snapper);
   }

   public void setIncludeHeightCost(boolean includeHeightCost)
   {
      this.includeHeightCost.set(includeHeightCost);
   }

   public void setIncludePitchAndRollCost(boolean includePitchAndRollCost)
   {
      this.includePitchAndRollCost.set(includePitchAndRollCost);
   }

   public FootstepCost buildCost()
   {
      includeHeightCost.setDefaultValue(false);
      includePitchAndRollCost.setDefaultValue(false);

      CompositeFootstepCost compositeFootstepCost = new CompositeFootstepCost();

      if (includeHeightCost.get())
         compositeFootstepCost.addFootstepCost(new HeightCost(footstepPlannerParameters.get().getCostParameters(), snapper.get()));

      if (includePitchAndRollCost.get())
         compositeFootstepCost.addFootstepCost(new PitchAndRollBasedCost(footstepPlannerParameters.get().getCostParameters(), snapper.get()));

      compositeFootstepCost.addFootstepCost(new DistanceAndYawBasedCost(footstepPlannerParameters.get()));

      FactoryTools.disposeFactory(this);

      return compositeFootstepCost;
   }

}
