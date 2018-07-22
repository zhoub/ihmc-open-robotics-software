package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointVelocityIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.PrivilegedConfigurationCalculator;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointControlParameters;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedJointSpaceManager
{
   private static final double VMC_VISCOUS_DAMPING = 1.0;

   private static final double POSITION_LIMIT_DAMPING = 10.0;
   private static final double POSITION_LIMIT_STIFFNESS = 100.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OneDoFJoint[] controlledJoints;

   private final VirtualModelControlCommandList virtualModelControlCommandList = new VirtualModelControlCommandList();
   private final JointLimitEnforcementCommand jointLimitEnforcementCommand = new JointLimitEnforcementCommand();
   private final JointTorqueCommand vmcJointDampingCommand = new JointTorqueCommand();

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final JointLimitEnforcementMethodCommand idJointLimitEnforcementCommand = new JointLimitEnforcementMethodCommand();

   private final InverseKinematicsCommandList inverseKinematicsCommandList = new InverseKinematicsCommandList();
   private final JointVelocityIntegrationCommand ikJointIntegrationCommand = new JointVelocityIntegrationCommand();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final PrivilegedConfigurationCalculator privilegedConfigurationCalculator;

   private final YoDouble vmcJointViscousDamping = new YoDouble("vmcJointViscousDamping", registry);
   private final YoDouble jointPositionLimitDamping = new YoDouble("jointPositionLimitDamping", registry);
   private final YoDouble jointPositionLimitStiffness = new YoDouble("jointPositionLimitStiffness", registry);

   private final YoDouble ikVelocityIntegrationBreakFrequency = new YoDouble("ikVelocityIntegrationBreakFrequency", registry);
   private final YoDouble ikAccelerationDifferentiationBreakFrequency = new YoDouble("ikAccelerationDifferentiationBreakFrequency", registry);
   
   private final QuadrupedWheelManager quadrupedWheelManager;
   

   public QuadrupedJointSpaceManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      controlledJoints = controllerToolbox.getFullRobotModel().getControllableOneDoFJoints();
      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      privilegedConfigurationCalculator = runtimeEnvironment.getPrivilegedConfigurationCalculator();
      
      quadrupedWheelManager = new QuadrupedWheelManager(controllerToolbox, registry);
      
      vmcJointViscousDamping.set(VMC_VISCOUS_DAMPING);
      jointPositionLimitDamping.set(POSITION_LIMIT_DAMPING);
      jointPositionLimitStiffness.set(POSITION_LIMIT_STIFFNESS);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         ikJointIntegrationCommand.addJointToComputeDesiredPositionFor(controlledJoint);
      }

      ikVelocityIntegrationBreakFrequency.set(0.1);
      ikAccelerationDifferentiationBreakFrequency.set(5.0);

      for (int i = 0; i < ikJointIntegrationCommand.getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         ikJointIntegrationCommand.setJointMaxima(i, 1.0, 100.0);
         ikJointIntegrationCommand
               .setBreakFrequencies(i, ikVelocityIntegrationBreakFrequency.getDoubleValue(), ikAccelerationDifferentiationBreakFrequency.getDoubleValue());
      }
      
      QuadrupedJointControlParameters jointControlParameters = controllerToolbox.getJointControlParameters();
      String[] jointNamesRestrictiveLimits = jointControlParameters.getJointsWithRestrictiveLimits();
      JointLimitParameters limitParameters = jointControlParameters.getJointLimitParametersForJointsWithRestictiveLimits();
      OneDoFJoint[] jointsWithRestrictiveLimit = ScrewTools.filterJoints(ScrewTools.findJointsWithNames(controlledJoints, jointNamesRestrictiveLimits),
                                                                         OneDoFJoint.class);
      for (OneDoFJoint joint : jointsWithRestrictiveLimit)
      {
         if (limitParameters == null)
         {
            throw new RuntimeException("Must define joint limit parameters if using joints with restrictive limits.");
         }
         idJointLimitEnforcementCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      vmcJointDampingCommand.clear();
      jointLimitEnforcementCommand.clear();
      quadrupedWheelManager.update();

      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];

         vmcJointDampingCommand.addJoint(joint, -vmcJointViscousDamping.getDoubleValue() * joint.getQd());
         jointLimitEnforcementCommand.addJoint(joint, jointPositionLimitStiffness.getDoubleValue(), jointPositionLimitDamping.getDoubleValue());

         ikJointIntegrationCommand
               .setBreakFrequencies(i, ikVelocityIntegrationBreakFrequency.getDoubleValue(), ikAccelerationDifferentiationBreakFrequency.getDoubleValue());
      }
      
      if(privilegedConfigurationCalculator != null)
      {
         privilegedConfigurationCalculator.calculate(privilegedConfigurationCommand);
      }
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return quadrupedWheelManager.getFeedbackControlCommand();
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      virtualModelControlCommandList.clear();
      virtualModelControlCommandList.addCommand(jointLimitEnforcementCommand);
      virtualModelControlCommandList.addCommand(vmcJointDampingCommand);

      return virtualModelControlCommandList;
   }

   public InverseKinematicsCommand<?> getInverseKinematicsCommand()
   {
      inverseKinematicsCommandList.clear();
      inverseKinematicsCommandList.addCommand(ikJointIntegrationCommand);
      if(privilegedConfigurationCalculator != null)
      {
         inverseKinematicsCommandList.addCommand(privilegedConfigurationCommand);
      }

      return inverseKinematicsCommandList;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(idJointLimitEnforcementCommand);
      if(privilegedConfigurationCalculator != null)
      {
         inverseDynamicsCommandList.addCommand(privilegedConfigurationCommand);
      }
      return inverseDynamicsCommandList;
   }
}
