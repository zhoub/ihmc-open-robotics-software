package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

public class EdgeCost
{
   private final double cost;

   public EdgeCost(double cost)
   {
      this.cost = cost;
   }

   public double getEdgeCost()
   {
      return cost;
   }
}
