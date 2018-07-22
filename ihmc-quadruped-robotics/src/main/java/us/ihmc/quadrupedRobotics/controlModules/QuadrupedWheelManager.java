package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedWheelManager
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();
   private final QuadrantDependentList<OneDoFJoint> wheelJoints = new QuadrantDependentList<>();
   private final YoPDGains wheelGains;
   private final YoDouble wheelWeight;
   
   
   private final YoDouble desiredWheelVelocity;
   private final YoDouble wheelDiameter;
   private final double controlDT;
   
   public QuadrupedWheelManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      controlDT = controllerToolbox.getRuntimeEnvironment().getControlDT();
      desiredWheelVelocity = new YoDouble("desiredWheelVelocity", registry);
      wheelDiameter = new YoDouble("wheelDiameter", registry);
      wheelDiameter.set(0.11);
      wheelWeight = new YoDouble("wheelQPWeight", registry);
      wheelWeight.set(1.0);
      
      wheelGains = new YoPDGains("wheelGains", registry);
      wheelGains.setKd(30.0);
      wheelGains.setKp(5.0);
      
      FullQuadrupedRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         OneDoFJoint wheelJoint = fullRobotModel.getLegJoint(robotQuadrant, LegJointName.ANKLE_ROLL);
         wheelJoints.put(robotQuadrant, wheelJoint);
         jointspaceFeedbackControlCommand.addJoint(wheelJoint, 0.0, 0.0, 0.0);
      }
      parentRegistry.addChild(registry);
   }
   
   public void update()
   {
      jointspaceFeedbackControlCommand.clear();
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         OneDoFJoint oneDoFJoint = wheelJoints.get(robotQuadrant);
         double desiredVelocity = desiredWheelVelocity.getDoubleValue() / wheelDiameter.getDoubleValue();
         double desiredPosition = oneDoFJoint.getQ() + desiredVelocity * controlDT;
         jointspaceFeedbackControlCommand.addJoint(oneDoFJoint, desiredPosition, desiredVelocity, 0.0);
         jointspaceFeedbackControlCommand.setWeightForSolver(wheelWeight.getDoubleValue());
         jointspaceFeedbackControlCommand.setGains(wheelGains);
         
      }
   }
   
   public JointspaceFeedbackControlCommand getFeedbackControlCommand()
   {
      return jointspaceFeedbackControlCommand;
   }
}
