package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import java.util.List;

import us.ihmc.pathPlanning.PlannerPlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface NavigableRegionFilter
{
   boolean isPlanarRegionNavigable(PlanarRegion query, List<PlanarRegion> allOtherRegions);
   boolean isPlanarRegionNavigable(PlannerPlanarRegion query, List<PlannerPlanarRegion> allOtherRegions);
}
