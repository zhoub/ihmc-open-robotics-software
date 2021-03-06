package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ControllerCoreOutput implements ControllerCoreOutputReadOnly
{
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final FrameVector3D linearMomentumRate = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final RootJointDesiredConfigurationData rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();
   private final JointDesiredOutputList lowLevelOneDoFJointDesiredDataHolder;

   public ControllerCoreOutput(CenterOfPressureDataHolder centerOfPressureDataHolder, OneDoFJointBasics[] controlledOneDoFJoints, JointDesiredOutputList lowLevelControllerOutput)
   {
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      linearMomentumRate.setToNaN(ReferenceFrame.getWorldFrame());
      if (lowLevelControllerOutput != null)
         lowLevelOneDoFJointDesiredDataHolder = lowLevelControllerOutput;
      else
         lowLevelOneDoFJointDesiredDataHolder = new JointDesiredOutputList(controlledOneDoFJoints);
   }

   public void setDesiredCenterOfPressure(FramePoint2D cop, RigidBodyBasics rigidBody)
   {
      centerOfPressureDataHolder.setCenterOfPressure(cop, rigidBody);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint2D copToPack, RigidBodyBasics rigidBody)
   {
      centerOfPressureDataHolder.getCenterOfPressure(copToPack, rigidBody);
   }

   public void setLinearMomentumRate(FrameVector3DReadOnly linearMomentumRate)
   {
      this.linearMomentumRate.set(linearMomentumRate);
   }

   public void setAndMatchFrameLinearMomentumRate(FrameVector3DReadOnly linearMomentumRate)
   {
      this.linearMomentumRate.setIncludingFrame(linearMomentumRate);
      this.linearMomentumRate.changeFrame(ReferenceFrame.getWorldFrame());
   }

   @Override
   public void getLinearMomentumRate(FrameVector3D linearMomentumRateToPack)
   {
      linearMomentumRateToPack.setIncludingFrame(linearMomentumRate);
   }

   public void setRootJointDesiredConfigurationData(RootJointDesiredConfigurationDataReadOnly rootJointDesiredConfigurationData)
   {
      this.rootJointDesiredConfigurationData.set(rootJointDesiredConfigurationData);
   }

   @Override
   public RootJointDesiredConfigurationDataReadOnly getRootJointDesiredConfigurationData()
   {
      return rootJointDesiredConfigurationData;
   }

   public void setLowLevelOneDoFJointDesiredDataHolder(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      this.lowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public JointDesiredOutputListReadOnly getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
