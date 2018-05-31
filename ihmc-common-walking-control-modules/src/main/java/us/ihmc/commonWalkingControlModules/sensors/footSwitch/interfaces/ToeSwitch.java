package us.ihmc.commonWalkingControlModules.sensors.footSwitch.interfaces;

import us.ihmc.robotics.sensors.FootSwitchInterface;

public interface ToeSwitch extends FootSwitchInterface
{
   public abstract boolean hasToeHitGround();

   public void resetToeSwitch();
}
