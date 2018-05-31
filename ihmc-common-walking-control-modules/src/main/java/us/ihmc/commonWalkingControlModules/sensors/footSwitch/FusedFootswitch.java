package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.ArrayList;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Allows you to combine foot switches and assign weights for voting.
 * This is useful if you want to something like combine the Kinematics 
 * Based foot switch and the ComputedForceBasedFootSwitch
 */
public class FusedFootswitch implements FootSwitchInterface
{
   private final YoVariableRegistry registry;
   private final ArrayList<FootSwitchInterface> touchdownDetectors = new ArrayList<>();
   private FootSwitchInterface footSwitchUsedForWrench;
   
   /**
    * The weight of each touchdown detector
    */
   private final TDoubleArrayList touchdownDetectorWeights = new TDoubleArrayList();
   
   /**
    * Should be a number between 0.0 and 1.0 that dictates when the footswitch will return true based on accumulated voting weights
    */
   private final YoDouble activationThreshold; 
   private final ReferenceFrame soleFrame;
   
   //Used to avoid GC, not useful state
   private final FramePoint2D copComputationHelper = new FramePoint2D();

   public FusedFootswitch(String prefix, ReferenceFrame soleFrame, double activationThreshold, YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      String name = prefix + "FusedFootswitch";
      this.registry = new YoVariableRegistry(name);
      this.activationThreshold = new YoDouble(name + "Activation", registry);
      this.activationThreshold.set(activationThreshold);
      
      parentRegistry.addChild(registry);
   }
   
   /**
    * Adds a touchdown detector to this fused foot switch
    * @param touchdownDetector The touchdown detector / footswitchg interface 
    * @param weight the scalar applied to the foot switch output for voting
    */
   public void addTouchdownDetector(FootSwitchInterface touchdownDetector, double weight)
   {
      touchdownDetectors.add(touchdownDetector);
      touchdownDetectorWeights.add(weight);
   }
   
   /**
    * The Fused Foot Switch only uses one foot switch for computing foot wrench. 
    * @param touchdownDetector
    */
   public void setTouchdownDetectorUsedToCalculateFootWrench(FootSwitchInterface touchdownDetector)
   {
      footSwitchUsedForWrench = touchdownDetector;
   }
   
   /**
    * Called internally to ensure the sum of the weights = 1.0
    * Will freak out if all weights are 0.0
    */
   private void normalizeWeights()
   {
      double sum = 0;
      for(int i = 0; i < touchdownDetectorWeights.size(); i++)
      {
         sum += touchdownDetectorWeights.get(i);
      }
      
      if(Math.abs(sum - 1.0) > 1e-3)
      {
         for(int i = 0; i < touchdownDetectorWeights.size(); i++)
         {
            touchdownDetectorWeights.set(i, touchdownDetectorWeights.get(i) / sum);
         }
      }
   }

   @Override
   public void reset()
   {
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         footSwitch.reset();
      }
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      normalizeWeights();
      
      double sum = 0.0;
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         if(footSwitch.getForceMagnitudePastThreshhold())
         {
            sum += touchdownDetectorWeights.get(i);
         }
      }
      return sum > activationThreshold.getDoubleValue();
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         footSwitch.setFootContactState(hasFootHitGround);
      }
   }

   @Override
   public boolean hasFootHitGround()
   {
      normalizeWeights();
      
      double sum = 0.0;
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         if(footSwitch.hasFootHitGround())
         {
            sum += touchdownDetectorWeights.get(i);
         }
      }
      return sum > activationThreshold.getDoubleValue();
   }

   /**
    * Computes the foot load percentage based on a weighted average of the foot switch interfaces
    * If a footswitch returns NaN or infinite the valid sources are weighted higher
    */
   @Override
   public double computeFootLoadPercentage()
   {
      normalizeWeights();
      
      double sum = 0.0;
      double accumulatedWeight = 0.0;
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         double loadPercentage = footSwitch.computeFootLoadPercentage();
         double weight = touchdownDetectorWeights.get(i);
         if(Double.isFinite(loadPercentage))
         {
            sum += loadPercentage * weight;
            accumulatedWeight += weight;
         }
      }
      
      if(accumulatedWeight > 0.0)
      {
         sum += (1.0 - accumulatedWeight) * sum / accumulatedWeight;
      }
      
      return Math.max(1.0, sum);
   }

   /**
    * Computes a weighted average of the CoP using the attached foot switches
    * If a footswitch returns NaNs the other sources are weighted higher
    */
   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      
      normalizeWeights();
      
      copToPack.setToZero(soleFrame);
      double accumulatedWeight = 0.0;
      
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         footSwitch.computeAndPackCoP(copComputationHelper);
         double weight = touchdownDetectorWeights.get(i);
         
         if(!copComputationHelper.containsNaN())
         {
            copComputationHelper.changeFrame(soleFrame);
            copComputationHelper.scale(weight);
            copToPack.add(copComputationHelper);
            accumulatedWeight+= weight;
         }
      }
      
      double unnacountedWeight = (1.0 - accumulatedWeight);
      if(unnacountedWeight > 1e-3)
      {
         copComputationHelper.setIncludingFrame(copToPack);
         copComputationHelper.scale(unnacountedWeight * copToPack.distanceFromOrigin() / accumulatedWeight);
         copToPack.add(copComputationHelper);
      }
   }

   @Override
   public void updateCoP()
   {
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         footSwitch.updateCoP();
      }
   }

   /**
    * Computes the foot wrench using the attached foot switch
    */
   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footSwitchUsedForWrench.computeAndPackFootWrench(footWrenchToPack);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return soleFrame;
   }

   @Override
   public void trustFootSwitch(boolean trustFootSwitch)
   {
      for(int i = 0; i < touchdownDetectors.size(); i++)
      {
         FootSwitchInterface footSwitch = touchdownDetectors.get(i);
         footSwitch.trustFootSwitch(trustFootSwitch);
      }
   }
}
