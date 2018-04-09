package us.ihmc.quadrupedRobotics.output;

import us.ihmc.sensorProcessing.outputProcessors.RobotOutputProcessor;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class StateChangeSmootherComponent implements RobotOutputProcessor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleParameter slopTimeParameter = new DoubleParameter("stateChangeSmootherSlopTime", registry, 0.0);
   private final DoubleParameter slopBreakFrequencyParameter = new DoubleParameter("stateChangeSmootherSlopBreakFrequency", registry, 1000.0);

   private final ArrayList<OneDoFJoint> allJoints = new ArrayList<>();
   private final LinkedHashMap<OneDoFJoint, AlphaFilteredYoVariable> jointTorquesSmoothedAtStateChange = new LinkedHashMap<>();
   private final YoDouble alphaJointTorqueForStateChanges = new YoDouble("alphaJointTorqueForStateChanges", registry);

   private final AtomicBoolean hasHighLevelControllerStateChanged = new AtomicBoolean(false);
   private final YoDouble timeAtHighLevelControllerStateChange = new YoDouble("timeAtControllerStateChange", registry);
   private final double controlDT;
   private final YoDouble controlTime;

   private JointDesiredOutputList jointDesiredOutputList;

   public StateChangeSmootherComponent(YoDouble controllerTime, double controlDT)
   {
      this.controlDT = controlDT;
      this.controlTime = controllerTime;

      alphaJointTorqueForStateChanges.set(0.0);
      timeAtHighLevelControllerStateChange.set(Double.NEGATIVE_INFINITY);
   }

   @Override
   public void setLowLevelControllerOutput(FullRobotModel fullRobotModel, JointDesiredOutputList jointDesiredOutputList)
   {
      this.jointDesiredOutputList = jointDesiredOutputList;

      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint oneDoFJoint = joints[i];
         String jointName = oneDoFJoint.getName();
         allJoints.add(oneDoFJoint);

         AlphaFilteredYoVariable jointTorqueSmoothedAtStateChange = new AlphaFilteredYoVariable("smoothed_tau_" + jointName, registry,
               alphaJointTorqueForStateChanges);
         jointTorquesSmoothedAtStateChange.put(oneDoFJoint, jointTorqueSmoothedAtStateChange);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void processAfterController(long timestamp)
   {
      if (hasHighLevelControllerStateChanged.get())
      {
         hasHighLevelControllerStateChanged.set(false);
         timeAtHighLevelControllerStateChange.set(controlTime.getDoubleValue());
      }

      double currentTime = controlTime.getDoubleValue();
      double deltaTime = Math.max(currentTime - timeAtHighLevelControllerStateChange.getDoubleValue(), 0.0);

      if (deltaTime < slopTimeParameter.getValue())
      {
         double breakFrequencyInHz = slopBreakFrequencyParameter.getValue() * (deltaTime / slopTimeParameter.getValue());
         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyInHz, controlDT);
         alphaJointTorqueForStateChanges.set(alpha);
      }
      else
      {
         alphaJointTorqueForStateChanges.set(0.0);
      }

      for (int i = 0; i < allJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = allJoints.get(i);
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint);
         double tau = jointDesiredOutput.getDesiredTorque();
         AlphaFilteredYoVariable smoothedJointTorque = jointTorquesSmoothedAtStateChange.get(oneDoFJoint);
         smoothedJointTorque.update(tau);
         jointDesiredOutput.setDesiredTorque(smoothedJointTorque.getDoubleValue());
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public <K extends Enum<K>> StateChangedListener<K> createFiniteStateMachineStateChangedListener()
   {
      return (from, to) -> hasHighLevelControllerStateChanged.set(true);
   }

   public ControllerStateChangedListener createControllerStateChangedListener()
   {
      ControllerStateChangedListener controllerStateChangedListener = new ControllerStateChangedListener()
      {
         @Override
         public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
         {
            hasHighLevelControllerStateChanged.set(true);
         }
      };

      return controllerStateChangedListener;
   }
}
