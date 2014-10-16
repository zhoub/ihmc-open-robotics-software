package us.ihmc.humanoidBehaviors.behaviors;

import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class LocalizationBehavior extends BehaviorInterface {
	
	private final FullRobotModel fullRobotModel;
	private ScriptBehavior scriptBehavior;
	//private String scriptName = "us/ihmc/atlas/scripts/TestWalkingLocalAll_LocalizationTest.xml";
	private String scriptName = "us/ihmc/atlas/scripts/SimpleFlatGroundScript.xml";
	private int i = 0;
	private RigidBodyTransform scriptObjectTransformToWorld = null;
	private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
	private ScriptBehaviorInputPacket receivedScriptBehavior;
	private boolean firstRun = false;
	
	public LocalizationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
	   {
	      super(outgoingCommunicationBridge);
	      
	      this.fullRobotModel = fullRobotModel;
	      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
	      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();
	      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
	   }

	@Override
	public void doControl() 
	{
		checkIfScriptBehaviorInputPacketReceived();
		
//		if (i == 0)
//		{
//			System.out.println("Starting iteration " + i + " of the localization test.");
//			i++;
//			reloadScriptBehavior();
//		}
		
		if ((scriptBehavior.isDone() && i < 10) || firstRun)
		{
			System.out.println("Starting iteration " + i + " of the localization test.");
			i++;
			reloadScriptBehavior();
			firstRun = false; 
		}

		scriptBehavior.doControl();
	}
	
	private void checkIfScriptBehaviorInputPacketReceived() 
	{
		if (scriptBehaviorInputPacketListener.isNewPacketAvailable()) 
		{
			firstRun = true;
			System.out.println("New packet available. Loading inputs...");
			receivedScriptBehavior = scriptBehaviorInputPacketListener.getNewestPacket();
			scriptObjectTransformToWorld = (receivedScriptBehavior.getReferenceTransform());
		}
	}

	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(
			Object object) {
		// TODO Auto-generated method stub

	}

	@Override
	protected void passReceivedControllerObjectToChildBehaviors(Object object) {
		// TODO Auto-generated method stub

	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub

	}

	@Override
	public void enableActions() {
		// TODO Auto-generated method stub

	}

	@Override
	public void pause() {
		// TODO Auto-generated method stub

	}

	@Override
	public void resume() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean isDone() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void finalize() {
		// TODO Auto-generated method stub

	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean hasInputBeenSet() {
		// TODO Auto-generated method stub
		return false;
	}
	
	private void reloadScriptBehavior() {
		scriptBehavior.finalize();
		scriptBehavior.initialize();
		scriptBehavior.setScriptInputs(scriptName,scriptObjectTransformToWorld);
		
	}

}
