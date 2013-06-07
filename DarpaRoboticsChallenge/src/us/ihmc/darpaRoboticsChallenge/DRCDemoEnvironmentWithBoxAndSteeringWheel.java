package us.ihmc.darpaRoboticsChallenge;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.ContactableToroidRobot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObject;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.Contactable;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.VehicleObject;
import us.ihmc.darpaRoboticsChallenge.controllers.SteeringWheelDisturbanceController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.Box3d;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject combinedTerrainObject;

   private final ArrayList<Robot> environmentRobots = new ArrayList<Robot>();
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();
   private final ContactableToroidRobot steeringWheelRobot;

   public DRCDemoEnvironmentWithBoxAndSteeringWheel(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DRCVehicleModelObjects drcVehicleModelObjects = new DRCVehicleModelObjects();


      Transform3D steeringWheelTransform = drcVehicleModelObjects.getTransform(VehicleObject.STEERING_WHEEL);


      /*
       * Quick estimates from 3D files:
       */
      double externalSteeringWheelRadius = 0.173;
      double internalSteeringWheelRadius = 0.143;

      double toroidRadius = (externalSteeringWheelRadius - internalSteeringWheelRadius) / 2.0;
      double steeringWheelRadius = (externalSteeringWheelRadius + internalSteeringWheelRadius) / 2.0;

      combinedTerrainObject = createCombinedTerrainObject(steeringWheelTransform);

      double mass = 1.0;
      steeringWheelRobot = new ContactableToroidRobot("steeringWheel", steeringWheelTransform, steeringWheelRadius, toroidRadius, mass);
      steeringWheelRobot.setDamping(2.0);
      steeringWheelRobot.createAvailableContactPoints(1, 30, 0.005, false);
      contactables.add(steeringWheelRobot);
      environmentRobots.add(steeringWheelRobot);
   }

   private CombinedTerrainObject createCombinedTerrainObject(Transform3D steeringWheelTransform)
   {
      CombinedTerrainObject terrainObject = new CombinedTerrainObject("carSeatBox");

      Vector3d steeringWheelTranslation = new Vector3d();
      steeringWheelTransform.get(steeringWheelTranslation);

      // mud_seat
//      addBox(-0.1, 0.0, 0.81, 0.0, 0.0, 0.0, 0.6, 1.15, 0.1, terrainObject);

      // seat
      addBox(-0.1, 0.0, 0.56, 0.0, 0.0, 0.0, 0.6, 1.22, 0.5, terrainObject);

      // seat_back
      addBox(-0.3, 0.0, 1.125, 0.0, -0.2, 0.0, 0.06, 1.0, 0.4, terrainObject);

      // chassis_bottom
      addBox(0.1, 0.0, 0.37, 0.0, 0.0, 0.0, 1.5, 1.34, 0.06, terrainObject);

      // engine
      addBox(1.05, 0.0, 0.7, 0.0, 0.0, 0.0, 0.58, 1.0, 0.8, terrainObject);

      // ground
      terrainObject.addBox(-1.0, -1.0, 1.0, 1.0, -0.05, 0.0, YoAppearance.DarkGray());

      return terrainObject;
   }

   private void addBox(double x, double y, double z, double roll, double pitch, double yaw, double sizeX, double sizeY, double sizeZ, CombinedTerrainObject terrainObject)
   {
      Transform3D transform = new Transform3D();
      transform.setEuler(new Vector3d(roll, pitch, yaw));
      transform.setTranslation(new Vector3d(x, y, z));
      Box3d box = new Box3d(transform, sizeX, sizeY, sizeZ);
      terrainObject.addRotatableBox(box, YoAppearance.DarkGray());
   }

   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>(environmentRobots);
   }

   public void addContactPoints(ExternalForcePoint[] contactPoints)
   {
      for (ExternalForcePoint contactPoint : contactPoints)
      {
         this.contactPoints.add(contactPoint);
      }
   }

   public void createAndSetContactControllerToARobot()
   {
      // add contact controller to any robot so it gets called
      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(contactables);
      environmentRobots.get(0).setController(contactController);
   }
   
   public void activateDisturbanceControllerOnSteeringWheel(YoFunctionGeneratorMode disturbanceMode)
   {
      SteeringWheelDisturbanceController controller = new SteeringWheelDisturbanceController(steeringWheelRobot, disturbanceMode);
      steeringWheelRobot.setController(controller);
   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      for (Contactable contactable : contactables)
      {
         if (contactable instanceof SelectableObject)
         {
            ((SelectableObject) contactable).addSelectedListeners(selectedListener);
         }
      }
   }
}
