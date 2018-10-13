package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.footstepPlanning.tools.FootstepPlannerDataExporter;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryMessager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.PlannerTypeTopic;

public abstract class FootstepPlannerDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;

   protected FootstepPlannerUI ui = null;
   protected SharedMemoryMessager messager = null;

   protected final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   protected final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);

   protected AtomicReference<FootstepPlan> uiFootstepPlanReference;
   protected AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   protected AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   protected AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   protected final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);

   public abstract FootstepPlannerType getPlannerType();

   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerSharedMemoryAPI.API);

      setupInternal();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      if (VISUALIZE)
      {
         createUI(messager);
      }

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);
   }

   public void setupInternal()
   {

   }

   @Test(timeout = 500000)
   @ContinuousIntegrationTest(estimatedDuration = 13.0)
   public void testDatasetsWithoutOcclusion()
   {
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationTest(estimatedDuration = 13.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(dataset -> runAssertions(dataset));
   }

   protected void runAssertionsOnAllDatasetsWithoutOcclusions(DatasetTestRunner datasetTestRunner)
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerDataExporter.class);

      runAssertionsOnAllDatasets(datasetTestRunner, allDatasets);
   }

   protected void runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(DatasetTestRunner datasetTestRunner)
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusionsInDevelopment(FootstepPlannerDataExporter.class);

      runAssertionsOnAllDatasets(datasetTestRunner, allDatasets);
   }

   protected void runAssertionsOnAllDatasets(DatasetTestRunner datasetTestRunner, List<FootstepPlannerUnitTestDataset> allDatasets)
   {
      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      int numberOfFailingDatasets = 0;
      int numberOfTotalDatasets = 0;
      String errorMessages = "";

      int currentDatasetIndex = 0;
      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      // Randomizing the regionIds so the viz is better
      Random random = new Random(324);
      allDatasets.stream().map(FootstepPlannerUnitTestDataset::getPlanarRegionsList).map(PlanarRegionsList::getPlanarRegionsAsList)
                 .forEach(regionsList -> regionsList.forEach(region -> region.setRegionId(random.nextInt())));

      FootstepPlannerUnitTestDataset dataset = allDatasets.get(currentDatasetIndex);

      List<String> dataSetNames = new ArrayList<>();
      while (dataset != null)
      {
         if (DEBUG)
         {
            PrintTools.info("Processing file: " + dataset.getDatasetName());
         }

         boolean hasType = false;
         for (FootstepPlannerType type : dataset.getTypes())
         {
            if (getPlannerType() == type)
               hasType = true;
         }

         if (hasType)
         {
            dataSetNames.add(dataset.getDatasetName());
            String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
            if (!errorMessagesForCurrentFile.isEmpty())
               numberOfFailingDatasets++;
            errorMessages += errorMessagesForCurrentFile;
            numberOfTotalDatasets++;
         }

         currentDatasetIndex++;
         if (currentDatasetIndex < allDatasets.size())
            dataset = allDatasets.get(currentDatasetIndex);
         else
            dataset = null;

         ThreadTools.sleep(100); // Apparently need to give some time for the prints to appear in the right order.
      }

      Assert.assertTrue("Number of failing datasets: " + numberOfFailingDatasets + " out of " + numberOfTotalDatasets + ". Errors:" + errorMessages,
                        errorMessages.isEmpty());

      PrintTools.info("Passed tests: ");
      for (String name : dataSetNames)
         PrintTools.info(name);
   }

   public String runAssertions(FootstepPlannerUnitTestDataset dataset)
   {
      submitDataSet(dataset);

      return findPlanAndAssertGoodResult(dataset);
   }

   public void runAssertionsOnDataset(DatasetTestRunner datasetTestRunner, String datasetName)
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerDataExporter.class);

      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      String errorMessages = "";

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      // Randomizing the regionIds so the viz is better
      Random random = new Random(324);
      allDatasets.stream().map(FootstepPlannerUnitTestDataset::getPlanarRegionsList).map(PlanarRegionsList::getPlanarRegionsAsList)
                 .forEach(regionsList -> regionsList.forEach(region -> region.setRegionId(random.nextInt())));

      FootstepPlannerUnitTestDataset dataset = null;
      for (FootstepPlannerUnitTestDataset datasetToQuery : allDatasets)
      {
         if (datasetToQuery.getDatasetName().equals(datasetName))
         {
            dataset = datasetToQuery;
            break;
         }
      }

      if (dataset == null)
         throw new RuntimeException("Dataset " + datasetName + " does not exist!");

      if (DEBUG)
      {
         PrintTools.info("Processing file: " + dataset.getDatasetName());
      }

      String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
      errorMessages += errorMessagesForCurrentFile;

      ThreadTools.sleep(100); // Apparently need to give some time for the prints to appear in the right order.

      Assert.assertTrue("Errors:" + errorMessages, errorMessages.isEmpty());
   }

   protected void packPlanningRequest(FootstepPlannerUnitTestDataset dataset, FootstepPlanningRequestPacket packet)
   {
      byte plannerType = getPlannerType().toByte();
      PlanarRegionsListMessage planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList());

      packet.getStanceFootPositionInWorld().set(dataset.getStart());
      packet.getGoalPositionInWorld().set(dataset.getGoal());
      packet.setRequestedFootstepPlannerType(plannerType);
      packet.getPlanarRegionsListMessage().set(planarRegions);

      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      packet.setTimeout(timeoutMultiplier * dataset.getTimeout(getPlannerType()));

      packet.setHorizonLength(Double.MAX_VALUE);

      if (dataset.hasGoalOrientation())
         packet.getGoalOrientationInWorld().set(dataset.getGoalOrientation());
      if (dataset.hasStartOrientation())
         packet.getStanceFootOrientationInWorld().set(dataset.getStartOrientation());

      if (DEBUG)
         PrintTools.info("Sending out planning request packet.");
   }

   protected void packPlanningRequest(FootstepPlannerUnitTestDataset dataset, SharedMemoryMessager messager)
   {
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartPositionTopic, dataset.getStart());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalPositionTopic, dataset.getGoal());
      messager.submitMessage(PlannerTypeTopic, getPlannerType());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerTimeoutTopic, timeMultiplier * dataset.getTimeout(getPlannerType()));

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerHorizonLengthTopic, Double.MAX_VALUE);

      if (dataset.hasGoalOrientation())
         messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, dataset.getGoalOrientation());
      if (dataset.hasStartOrientation())
         messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, dataset.getStartOrientation());

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.ComputePathTopic, true);

      if (DEBUG)
         PrintTools.info("Sending out planning request packet.");
   }

   protected void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      plannerResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));
      plannerReceivedPlan.set(true);
   }


   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         SimpleFootstep footstep = footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);

         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (int i = 0; i < footstepMessage.getPredictedContactPoints2d().size(); i++)
            foothold.addVertex(footstepMessage.getPredictedContactPoints2d().get(i));
         foothold.update();
         footstep.setFoothold(foothold);
      }

      return footstepPlan;
   }

   protected String assertPlansAreValid(String datasetName, FootstepPlanningResult expectedResult, FootstepPlanningResult actualResult,
                                        FootstepPlan expectedPlan, FootstepPlan actualPlan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning results for " + datasetName + " are not equal: " + expectedResult + " and " + actualResult + ".\n",
                                 expectedResult.equals(actualResult));

      errorMessage += assertPlanIsValid(datasetName, expectedResult, expectedPlan, goal);
      errorMessage += assertPlanIsValid(datasetName, actualResult, actualPlan, goal);

      if (actualResult.validForExecution())
      {
         errorMessage += areFootstepPlansEqual(actualPlan, expectedPlan);
      }

      return errorMessage;
   }

   protected String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning result for " + datasetName + " is invalid, result was " + result, result.validForExecution());

      if (result.validForExecution())
      {
         errorMessage += assertTrue(datasetName, datasetName + " did not reach goal.", PlannerTools.isGoalNextToLastStep(goal, plan));
      }

      return errorMessage;
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            PrintTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   private String areFootstepPlansEqual(FootstepPlan footstepPlanA, FootstepPlan footstepPlanB)
   {
      String errorMessage = "";

      if (footstepPlanA.getNumberOfSteps() != footstepPlanB.getNumberOfSteps())
      {
         errorMessage += "Plan A has " + footstepPlanA.getNumberOfSteps() + ", while Plan B has " + footstepPlanB.getNumberOfSteps() + ".\n";
      }

      for (int i = 0; i < Math.min(footstepPlanA.getNumberOfSteps(), footstepPlanB.getNumberOfSteps()); i++)
      {
         errorMessage += areFootstepsEqual(i, footstepPlanA.getFootstep(i), footstepPlanB.getFootstep(i));
      }

      return errorMessage;
   }

   private String areFootstepsEqual(int footstepNumber, SimpleFootstep footstepA, SimpleFootstep footstepB)
   {
      String errorMessage = "";

      if (!footstepA.getRobotSide().equals(footstepB.getRobotSide()))
      {
         errorMessage += "Footsteps " + footstepNumber + " are different robot sides: " + footstepA.getRobotSide() + " and " + footstepB.getRobotSide() + ".\n";
      }

      FramePose3D poseA = new FramePose3D();
      FramePose3D poseB = new FramePose3D();

      footstepA.getSoleFramePose(poseA);
      footstepB.getSoleFramePose(poseB);

      if (!poseA.epsilonEquals(poseB, 1e-5))
      {
         errorMessage += "Footsteps " + footstepNumber + " have different poses: \n \t" + poseA.toString() + "\n and \n\t " + poseB.toString() + ".\n";
      }

      if (!footstepA.epsilonEquals(footstepB, 1e-5))

      {
         errorMessage += "Footsteps " + footstepNumber + " are not equal: \n \t" + footstepA.toString() + "\n and \n\t " + footstepB.toString() + ".\n";
      }

      return errorMessage;
   }

   protected void createUI(SharedMemoryMessager messager)
   {
      ApplicationRunner.runApplication(new Application()
      {
         @Override
         public void start(Stage stage) throws Exception
         {
            ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
            ui.show();
         }

         @Override
         public void stop() throws Exception
         {
            ui.stop();
            Platform.exit();
         }
      });

      double maxWaitTime = 5.0;
      double totalTime = 0.0;
      long sleepDuration = 100;

      while (ui == null)
      {
         if (totalTime > maxWaitTime)
            throw new RuntimeException("Timed out waiting for the UI to start.");
         ThreadTools.sleep(sleepDuration);
         totalTime += Conversions.millisecondsToSeconds(sleepDuration);
      }
   }

   protected double totalTimeTaken;

   protected String waitForResult(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";
      long waitTime = 10;
      while (conditionChecker.checkCondition())
      {
         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting for a result.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   protected String validateResult(ConditionChecker conditionChecker, FootstepPlanningResult result, String prefix)
   {
      String errorMessage = "";

      if (!conditionChecker.checkCondition())
      {
         errorMessage += prefix + " failed to find a valid result. Result : " + result + "\n";
      }

      return errorMessage;
   }

   protected String waitForPlan(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";

      while (conditionChecker.checkCondition())
      {
         long waitTime = 10;

         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting on plan.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   protected void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the UI.");
         actualPlan.set(uiFootstepPlanReference.get());
      }

      if (uiReceivedResult.get() && uiPlanningResultReference.get() != null && actualResult.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + uiPlanningResultReference.get() + " from the UI.");
         actualResult.set(uiPlanningResultReference.get());
      }
   }

   protected void queryPlannerResults()
   {
      if (plannerReceivedPlan.get() && plannerPlanReference.get() != null && expectedPlan.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the planner.");
         expectedPlan.set(plannerPlanReference.get());
      }

      if (plannerReceivedPlan.get() && plannerResultReference.get() != null && expectedResult.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + plannerResultReference.get() + " from the planner.");
         expectedResult.set(plannerResultReference.get());
      }
   }

   public abstract void submitDataSet(FootstepPlannerUnitTestDataset dataset);

   public abstract String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset);

   protected static interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }

   protected static interface ConditionChecker
   {
      boolean checkCondition();
   }
}
