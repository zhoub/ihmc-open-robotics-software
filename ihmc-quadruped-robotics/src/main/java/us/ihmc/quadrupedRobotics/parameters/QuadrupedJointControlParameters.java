package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPDGains;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedJointControlParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final EnumParameter<JointDesiredControlMode> standPrepJointMode = new EnumParameter<>("standPrepJointMode", registry, JointDesiredControlMode.class,
                                                                                                 false, JointDesiredControlMode.POSITION);
   private final ParameterizedPDGains positionStandPrepGains;
   private final ParameterizedPDGains effortStandPrepGains;

   private final EnumParameter<JointDesiredControlMode> freezeJointMode = new EnumParameter<>("freezeJointMode", registry, JointDesiredControlMode.class, false,
                                                                                              JointDesiredControlMode.POSITION);
   private final ParameterizedPDGains positionFreezeGains;

   private final EnumParameter<JointDesiredControlMode> doNothingJointMode = new EnumParameter<>("doNothingJointMode", registry, JointDesiredControlMode.class,
                                                                                                 false, JointDesiredControlMode.POSITION);
   private final ParameterizedPDGains positionDoNothingGains;

   private final EnumParameter<JointDesiredControlMode> jointInitializationJointMode = new EnumParameter<>("jointInitializationJointMode", registry,
                                                                                                           JointDesiredControlMode.class, false,
                                                                                                           JointDesiredControlMode.POSITION);
   private final ParameterizedPDGains positionJointInitializationGains;

   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode;

   private final ParameterizedPDGains vmcLoadedGains;
   private final ParameterizedPDGains vmcUnloadedGains;
   private final ParameterizedPDGains idLoadedGains;
   private final ParameterizedPDGains idUnloadedGains;
   private final ParameterizedPDGains ikLoadedGains;
   private final ParameterizedPDGains ikUnloadedGains;

   public QuadrupedJointControlParameters(YoEnum<WholeBodyControllerCoreMode> controllerCoreMode, YoVariableRegistry parentRegistry)
   {
      this.controllerCoreMode = controllerCoreMode;
      PDGains ikStandPrep = new PDGains();
      ikStandPrep.setKp(1000.0);
      ikStandPrep.setKd(100.0);
      positionStandPrepGains = new ParameterizedPDGains("positionStandPrep", ikStandPrep, registry);

      PDGains effortStandPrep = new PDGains();
      effortStandPrep.setKp(2000.0);
      effortStandPrep.setKd(200.0);
      effortStandPrepGains = new ParameterizedPDGains("effortStandPrep", effortStandPrep, registry);

      PDGains positionFreeze = new PDGains();
      positionFreeze.setKp(2000.0);
      positionFreeze.setKd(200.0);
      positionFreezeGains = new ParameterizedPDGains("positionFreeze", positionFreeze, registry);

      PDGains positionDoNothing = new PDGains();
      positionDoNothing.setKp(1000.0);
      positionDoNothing.setKd(100.0);
      positionDoNothingGains = new ParameterizedPDGains("positionDoNothing", positionDoNothing, registry);

      PDGains positionJointInitialization = new PDGains();
      positionJointInitialization.setKp(2000.0);
      positionJointInitialization.setKd(200.0);
      positionJointInitializationGains = new ParameterizedPDGains("positionJointInitialization", positionJointInitialization, registry);

      PDGains vmcLoaded = new PDGains();
      vmcLoaded.setKp(0.0);
      vmcLoaded.setKd(0.0);
      vmcLoadedGains = new ParameterizedPDGains("vmcLoaded", vmcLoaded, registry);

      PDGains vmcUnloaded = new PDGains();
      vmcUnloaded.setKp(0.0);
      vmcUnloaded.setKd(0.0);
      vmcUnloadedGains = new ParameterizedPDGains("vmcUnloaded", vmcUnloaded, registry);
      
      PDGains idLoaded = new PDGains();
      idLoaded.setKp(0.0);
      idLoaded.setKd(0.0);
      idLoadedGains = new ParameterizedPDGains("idLoaded", idLoaded, registry);
      
      PDGains idUnloaded = new PDGains();
      idUnloaded.setKp(0.0);
      idUnloaded.setKd(0.0);
      idUnloadedGains = new ParameterizedPDGains("idUnloaded", idUnloaded, registry);
      
      PDGains ikLoaded = new PDGains();
      ikLoaded.setKp(500);
      ikLoaded.setKd(100.0);
      ikLoadedGains = new ParameterizedPDGains("ikLoaded", ikLoaded, registry);

      PDGains ikUnloaded = new PDGains();
      ikUnloaded.setKp(100.0);
      ikUnloaded.setKd(20.0);
      ikUnloadedGains = new ParameterizedPDGains("ikUnloaded", ikUnloaded, registry);

      parentRegistry.addChild(registry);
   }

   public JointDesiredControlMode getStandPrepJointMode()
   {
      return standPrepJointMode.getValue();
   }

   public PDGainsReadOnly getStandPrepJointGains()
   {
      switch (standPrepJointMode.getValue())
      {
      case POSITION:
         return positionStandPrepGains;
      case EFFORT:
         return effortStandPrepGains;
      default:
         throw new RuntimeException("The joint control mode " + standPrepJointMode.getValue() + " is not implemented for stand prep.");
      }
   }

   public JointDesiredControlMode getFreezeJointMode()
   {
      return freezeJointMode.getValue();
   }

   public PDGainsReadOnly getFreezeJointGains()
   {
      switch (freezeJointMode.getValue())
      {
      case POSITION:
         return positionFreezeGains;
      default:
         throw new RuntimeException("The joint control mode " + freezeJointMode.getValue() + " is not implemented for freeze.");
      }
   }

   public JointDesiredControlMode getDoNothingJointMode()
   {
      return doNothingJointMode.getValue();
   }

   public PDGainsReadOnly getDoNothingJointGains()
   {
      switch (doNothingJointMode.getValue())
      {
      case POSITION:
         return positionDoNothingGains;
      default:
         throw new RuntimeException("The joint control mode " + doNothingJointMode.getValue() + " is not implemented for do nothing.");
      }
   }

   public JointDesiredControlMode getJointInitializationJointMode()
   {
      return jointInitializationJointMode.getValue();
   }

   public PDGainsReadOnly getJointInitializationJointGains()
   {
      switch (jointInitializationJointMode.getValue())
      {
      case POSITION:
         return positionJointInitializationGains;
      default:
         throw new RuntimeException("The joint control mode " + jointInitializationJointMode.getValue() + " is not implemented for joint initialization.");
      }
   }

   public JointDesiredControlMode getSteppingJointMode()
   {
      switch (controllerCoreMode.getEnumValue())
      {
      case INVERSE_KINEMATICS:
         return JointDesiredControlMode.POSITION;
      case VIRTUAL_MODEL:
      case INVERSE_DYNAMICS:
         return JointDesiredControlMode.EFFORT;
      default:
         throw new RuntimeException("The controller core mode " + controllerCoreMode.getValue() + " is not implemented.");
      }
   }

   public PDGainsReadOnly getSteppingLoadedJointGains()
   {
      switch (controllerCoreMode.getEnumValue())
      {
      case INVERSE_KINEMATICS:
         return ikLoadedGains;
      case VIRTUAL_MODEL:
         return vmcLoadedGains;
      case INVERSE_DYNAMICS:
         return idLoadedGains;
      default:
         throw new RuntimeException("The controller core mode " + controllerCoreMode.getValue() + " is not implemented.");
      }
   }

   public PDGainsReadOnly getSteppingUnloadedJointGains()
   {
      switch (controllerCoreMode.getEnumValue())
      {
      case INVERSE_KINEMATICS:
         return ikUnloadedGains;
      case VIRTUAL_MODEL:
         return vmcUnloadedGains;
      case INVERSE_DYNAMICS:
         return idUnloadedGains;
      default:
         throw new RuntimeException("The controller core mode " + controllerCoreMode.getValue() + " is not implemented.");
      }
   }
   
   /**
    * Returns a list of joint that should use the more restrictive joint limit enforcement
    * in the QP. If the list is not empty the method {@link #getJointLimitParametersForJointsWithRestictiveLimits()}
    * must be overwritten to define the limit parameters.
    */
   public String[] getJointsWithRestrictiveLimits()
   {
      return new String[0];
   }

   /**
    * Returns parameters for joint limits that will be used with the joints defined in
    * {@link #getJointsWithRestrictiveLimits()}.
    */
   public JointLimitParameters getJointLimitParametersForJointsWithRestictiveLimits()
   {
      JointLimitParameters parameters = new JointLimitParameters();
      parameters.setMaxAbsJointVelocity(15.0);
      parameters.setJointLimitDistanceForMaxVelocity(30.0 * Math.PI/180.0);
      parameters.setJointLimitFilterBreakFrequency(15.0);
      parameters.setVelocityControlGain(30.0);
      return parameters;
   }
}
