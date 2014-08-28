package infraestructura;

import java.util.Vector;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.rosjava_geometry.Vector3;

public class MyNode implements NodeMain{

	/**
	 * @param args
	 */
	public MyNode() {
		// TODO Auto-generated constructor stub
	}
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onError(Node arg0, Throwable arg1) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode arg0) {
		// TODO Auto-generated method stub
				
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		
		
		return null;
	}

}