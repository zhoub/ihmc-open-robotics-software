package us.ihmc.valkyrie.controllerAPI;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   ValkyrieEndToEndHandTrajectoryMessageTest.class,
   ValkyrieEndToEndArmTrajectoryMessageTest.class,
   ValkyrieEndToEndArmDesiredAccelerationsMessageTest.class,
   ValkyrieEndToEndChestTrajectoryMessageTest.class,
   ValkyrieEndToEndHeadTrajectoryMessageTest.class,
   ValkyrieEndToEndNeckTrajectoryMessageTest.class,
   ValkyrieEndToEndNeckDesiredAccelerationsMessageTest.class,
   ValkyrieEndToEndWholeBodyTrajectoryMessageTest.class
})

public class ValkyrieEndToEndRigidBodyManagerSuite
{

}
