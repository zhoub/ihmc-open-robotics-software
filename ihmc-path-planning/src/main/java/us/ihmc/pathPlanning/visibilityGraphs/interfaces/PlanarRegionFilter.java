package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.pathPlanning.PlannerPlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface PlanarRegionFilter
{
   boolean isPlanarRegionRelevant(PlanarRegion planarRegion);
   boolean isPlanarRegionRelevant(PlannerPlanarRegion planarRegion);
}
