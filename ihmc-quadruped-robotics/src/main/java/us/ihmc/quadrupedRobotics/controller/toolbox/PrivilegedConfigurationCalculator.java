package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;

public interface PrivilegedConfigurationCalculator
{
   public void calculate(PrivilegedConfigurationCommand privilegedConfigurationCommand);

   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters();

}