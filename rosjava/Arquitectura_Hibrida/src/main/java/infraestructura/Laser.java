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
public class Laser extends Sensor{
	
	/**
	 * Posible atributo del nuevo sensor del robot: info.
	 * 
	 * TODO Incluir los atributos necesarios para representar el nuevo elemento del robot.
	 * 
	 */
	private int informacion;
	
	
	
	
	   private ConnectedNode node;
        Subscriber<std_msgs.String> subscriber;

        public ConnectedNode getNode() {
		return node;
	}


	public void setNode(ConnectedNode node) {
		this.node = node;
	}

/**
 * @autor Ariel Vernaza
 * Recibe la Informacion de la Camara procesada por el agente desde webots.
 * 
 * */
	
	/**
	 * TODO Incluir el constructor correspondiente para inicializar el nuevo sensor del robot.
	 */
	public Laser(final ConnectedNode node){
		super("laser");
		this.informacion = 0;
		this.node=node;
		subscriber = node.newSubscriber("/laser", std_msgs.String._TYPE);
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
                @Override
                  public void onNewMessage(std_msgs.String message) 
                      {
                       String[] temp2;
                       informacion =Integer.parseInt(message.getData().trim());			              		         
                      }
                  });
	}
	
	
	
	/**
	 * TODO Incluir mtodos de acceso asociados a los atributos definidos.
	 * 
	 * @return El valor de retorno
	 * 
	 */
	public int getInfo(){
		return this.informacion;
	}
	
	/**
	 * TODO Incluir otros mtodos necesarios para la clase.
	 * 
	 * @param info Un parmetro del mtodo
	 */
	public void setInfo(int info){
		this.informacion = info;
	}


	

}
