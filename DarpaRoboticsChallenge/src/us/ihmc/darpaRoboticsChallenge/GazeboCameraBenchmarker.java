package us.ihmc.darpaRoboticsChallenge;

import java.net.URI;
import java.net.URISyntaxException;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosTools;
import us.ihmc.utilities.keyboardAndMouse.RepeatingReleasedEventsFixer;

public class GazeboCameraBenchmarker extends AbstractNodeMain
{
	public static final String MASTER = "http://10.4.42.4:11311";
	public static final String TOPIC_NAME = "/image_raw";
	
	private Subscriber<sensor_msgs.Image> cameraSubscriber;
	private String topicName;
	
	private long initialTime;
	private double framesReceived = 0.0;
	
	public GazeboCameraBenchmarker(String topicName)
	{
		new RepeatingReleasedEventsFixer().install();
		
		this.topicName = topicName;
	}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("darpaRoboticsChallenge/GazeboCameraBenchmarker" + topicName);
	}
	
	public void onStart(ConnectedNode connectedNode)
	{
		setupSubscriber(connectedNode);
		
		setupCameraSubscriberListener();
	}
	
	private void setupCameraSubscriberListener()
	{
		cameraSubscriber.addMessageListener(new MessageListener<sensor_msgs.Image>()
		{
			public void onNewMessage(sensor_msgs.Image message)
			{
				if (framesReceived == 0.0)
					initialTime = System.currentTimeMillis();
				else
				{
					System.out.println("FPS: "+(framesReceived*1000.0)/(System.currentTimeMillis()-initialTime));
				}
				++framesReceived;
			}
			
		});
	}
	
	private void setupSubscriber(ConnectedNode connectedNode)
	{
		cameraSubscriber = connectedNode.newSubscriber(topicName, sensor_msgs.Image._TYPE);
	}
	
	public static void main(String[] args) throws URISyntaxException
	{	
		URI master = new URI(MASTER);
		
		try
		{
			GazeboCameraBenchmarker externalCamera = new GazeboCameraBenchmarker(TOPIC_NAME);
			NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
			NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(externalCamera, nodeConfiguration);
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}
}
