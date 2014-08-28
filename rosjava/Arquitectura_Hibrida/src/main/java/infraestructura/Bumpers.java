package infraestructura;

import org.ros.namespace.GraphName;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;


/**
 * Clase que hereda de Sensor y define las caractersticas 
 * propias de este tipo de sensor e implementa la funcionalidad 
 * relacionada con ellas.
 * 
 * @author Jos Luis Daz Cebrin
 * @version 1.0
 * 
 */
public class Bumpers extends Sensor{
	
	/**
	 * Nmero de bumpers frontales que posee el robot.
	 */
	private final int NUM_FRONT;
	
	/**
	 * Nmero de bumpers traseros que posee el robot.
	 */
	private final int NUM_REAR;
	
	/**
	 * Valor dado por el sensor de los bumpers.
	 */
	private int valor;
	
        private ConnectedNode node;
        Subscriber<std_msgs.String> subscriber;

        public ConnectedNode getNode() {
		return node;
	}


	public void setNode(ConnectedNode node) {
		this.node = node;
	}
	
	/**
	 * Inicializa el sensor con el nombre "bumpers" y el nmero de bumpers
	 * delanteros y traseros que posee el robot.
	 * 
	 * @param frontales El nmero de bumpers frontales que posee el robot
	 * @param traseros El nmero de bumpers traseros que posee el robot
	 * 
	 */
	public Bumpers(int frontales, int traseros,final ConnectedNode node){
		super("bumpers");
                this.node=node;
                subscriber = node.newSubscriber("/bumper_state", std_msgs.String._TYPE);
		this.NUM_FRONT = frontales;
		this.NUM_REAR = traseros;
		this.valor = 0;
                subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
                @Override
                  public void onNewMessage(std_msgs.String message) 
                      {
                       setValor(Integer.parseInt(message.getData().trim()));
                      // System.out.println("Valor del bumper: \"" + getValor() + "\"");
                      }
                  });
	}
	
	
	/**
	 * Devuelve el nmero de bumpers frontales del robot.
	 * 
	 * @return El nmero de bumpers frontales
	 * 
	 */
	public int getNumFrontales(){
		return this.NUM_FRONT;
	}
	
	/**
	 * Devuelve el nmero de bumpers traseros del robot.
	 * 
	 * @return El nmero de bumpers traseros
	 * 
	 */
	public int getNumTraseros(){
		return this.NUM_REAR;
	}
	
	/**
	 * Devuelve el valor detectado por los bumpers del robot.
	 * 
	 * @return El valor detectado
	 * 
	 */
	public int getValor(){
		return this.valor;
	}
	
	/**
	 * Asigna un valor detectado a los bumpers del robot.
	 */
	public void setValor(int val){
		this.valor = val;
	}

	
}
