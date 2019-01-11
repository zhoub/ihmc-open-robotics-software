package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

public class FlatGroundFootstepNodeSnapper extends FootstepNodeSnapper
{
   public FlatGroundFootstepNodeSnapper()
   {
      super(null);
   }

   @Override
   public FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
   {
      return FootstepNodeSnapData.identityData();
   }
}
