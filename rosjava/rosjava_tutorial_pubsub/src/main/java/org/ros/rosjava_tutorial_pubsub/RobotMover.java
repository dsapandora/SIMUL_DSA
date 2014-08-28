package org.ros.rosjava_tutorial_pubsub;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;

import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import org.ros.node.topic.Publisher;
import geometry_msgs.Twist;
import org.ros.rosjava_geometry.Vector3;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;

public class RobotMover  extends AbstractNodeMain {

   @Override
   public GraphName getDefaultNodeName() {
     return GraphName.of("mypackagename/OdemetryCollector");
   }
  
 
  public void onStart(final ConnectedNode node) {
    final Publisher<Twist> publisher =
        node.newPublisher("cmd_vel", "geometry_msgs/Twist");
    final Log log = node.getLog();

    log.info("Sleeping for 1s before starting.");
    try { Thread.sleep(1000); } catch(InterruptedException ie) { ie.printStackTrace(); }
    node.executeCancellableLoop(new CancellableLoop() {

    
        @Override
        protected void loop() throws InterruptedException {
        MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
	MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
	    	    
	Twist cmd =  publisher.newMessage();;
        log.info("Moving robot forward.");
        
        Vector3 arg0 =new Vector3(100.2, 100.2, 0.0) ; // move forward 0.2m/s for 1s
        geometry_msgs.Vector3 arg1 = messageFactory.newFromType(geometry_msgs.Vector3._TYPE); 
        cmd.setLinear(arg0.toVector3Message(arg1));	    	        
        publisher.publish(cmd);
        Thread.sleep(1000);

	//publisher.publish(new Twist()); // pause for 1s
	Thread.sleep(1000);

	log.info("Moving robot backward.");
        arg0 = new Vector3(-100.2, -100.2, 0.0) ; // move forward 0.2m/s for 1s
        cmd.setLinear(arg0.toVector3Message(arg1));	   	        
        publisher.publish(cmd);
	Thread.sleep(1000);

	//publisher.publish(new Twist()); // pause for 1s
	Thread.sleep(1000);
      }

    });}

  public void onError(final  ConnectedNode node, Throwable throwable)
  { 
    System.out.println("Hubo un error");
  }


 
  
  public void onShutdown(final ConnectedNode node) {
        MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
	MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
	    	    
	final Publisher<Twist> publisher = node.newPublisher("/cmd_vel", "geometry_msgs/Twist");
	final Log log = node.getLog();
	log.info("Shutdown called.  Making robot stop.");
	Twist cmd =  publisher.newMessage();;
        log.info("Moving robot forward.");
        Vector3 arg0 = new Vector3(0.0, 0.0, 0.0); 
        geometry_msgs.Vector3 arg1 = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
        cmd.setLinear(arg0.toVector3Message(arg1));	   	      
        publisher.publish(cmd);
  }

 

  
}
