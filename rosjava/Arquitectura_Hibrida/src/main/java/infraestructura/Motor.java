package infraestructura;

import geometry_msgs.Twist;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.Vector3;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

/**
 * Clase que hereda de Actuador y define las caractersticas 
 * propias de este tipo de actuador e implementa la funcionalidad 
 * relacionada con ellas.
 * 
 * @author Jos Luis Daz Cebrin
 * @version 1.0
 * 
 */

public class Motor extends Actuador{

	/**
	 * Velocidad de la rueda izquierda del robot.
	 */
	private int rueda_izda;
	/**
	 * Velocidad de la rueda derecha del robot.
	 */
	private int rueda_dcha;
	
	/**
	 * Valor del encoder de la rueda izquierda del robot.
	 */
	private double encoder_izdo;
	
	/**
	 * Valor del encoder de la rueda derecha del robot.
	 */
	private double encoder_dcho;
	
	/**
	 * Valor en centmetros de la distancia de odometra recorrida.
	 */
	private double distancia;
	
	/**
	 * Valor en grados del giro realizado, medido por odometra.
	 */
	private double giro;
	
	
	/**
	 * Inicializa el actuador con el nombre "motor" y una velocidad de
	 * 0 en cada rueda del robot.
	 */
	private ConnectedNode node;
	final Publisher<Twist> publisher; 
	Subscriber<std_msgs.String> distancia_dsa;
        Subscriber<std_msgs.String> giro_dsa;

	public ConnectedNode getNode() {
		return node;
	}


	public void setNode(ConnectedNode node) {
		this.node = node;
	}

        public void publicador() 
        {
                 MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
		 MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
		    	    
		Twist cmd =  publisher.newMessage();;
	        Vector3 arg0 =new Vector3( rueda_dcha,rueda_izda, 0.0) ; // move forward 0.2m/s for 1s
                
	        geometry_msgs.Vector3 arg1 = messageFactory.newFromType(geometry_msgs.Vector3._TYPE); 
	        cmd.setLinear(arg0.toVector3Message(arg1));	    	        
	        publisher.publish(cmd);
		System.out.println("LE HE ENVIADO A LAS RUEDAS: "+ rueda_dcha+" "+rueda_izda);	
        }
	public Motor(final ConnectedNode node){
		super("motor",node);
		this.node=node;
		publisher=node.newPublisher("/cmd_vel2", "geometry_msgs/Twist");
                distancia_dsa = node.newSubscriber("/odometria_state", std_msgs.String._TYPE);
                giro_dsa = node.newSubscriber("/TripOdometerDegrees_state", std_msgs.String._TYPE);
		this.rueda_izda = 0;
		this.rueda_dcha = 0;
		this.encoder_izdo = 0;
		this.encoder_dcho = 0;
		this.distancia = 0;
		this.giro = 0;
                distancia_dsa.addMessageListener(new MessageListener<std_msgs.String>() {
                @Override
                  public void onNewMessage(std_msgs.String message) 
                      {
                       setDistancia(Double.parseDouble(message.getData()));
                       System.out.println("Distancia: \"" + message.getData() + "\"");
                      }
                  });
                giro_dsa.addMessageListener(new MessageListener<std_msgs.String>() {
                @Override
                  public void onNewMessage(std_msgs.String message) 
                      {
                       setGiro(Double.parseDouble(message.getData()));
                       System.out.println("Giro: \"" + message.getData() + "\"");
                      }
                  });
                publicador();
		try { Thread.sleep(1000); } catch(InterruptedException ie) { ie.printStackTrace(); }
	  /*  node.executeCancellableLoop(new CancellableLoop() {

	    
	        @Override
	        protected void loop() throws InterruptedException {
	       
	      }

	    });	*/	
	}
	
	
	/**
	 * Devuelve el valor de la velocidad asignada a la rueda izquierda.
	 * 
	 * @return La velocidad de la rueda izquierda
	 * 
	 */
	public int getRuedaIzda(){
		return this.rueda_izda;
	}
	
	/**
	 * Devuelve el valor de la velocidad asignada a la rueda derecha.
	 * 
	 * @return La velocidad de la rueda derecha
	 * 
	 */
	public int getRuedaDcha(){
		return this.rueda_dcha;
	}
	
	/**
	 * Devuelve el valor del encoder izquierdo.
	 * 
	 * @return El valor del encoder izquierdo
	 * 
	 */
	public double getEncoderIzdo(){
		return this.encoder_izdo;
	}
	
	/**
	 * Devuelve el valor del encoder derecho.
	 * 
	 * @return El valor del encoder derecho
	 * 
	 */
	public double getEncoderDcho(){
		return this.encoder_dcho;
	}
	
	/**
	 * Devuelve la distancia recorrida por el robot, medida por odometra.
	 * 
	 * @return La distancia recorrida
	 * 
	 */
	public double getDistancia(){
		return this.distancia;
	}
	
	/**
	 * Devuelve el giro realizado por el robot, medido por odometra.
	 * 
	 * @return El giro realizado
	 * 
	 */
	public double getGiro(){
		return this.giro;
	}
	
	/**
	 * Asigna una velocidad a cada una de las ruedas del motor.
	 * 
	 * @param izda La velocidad a asignar a la rueda izquierda
	 * @param dcha La velocidad a asignar a la rueda derecha
	 * 
	 */
	public void setRuedas(int izda, int dcha){
		this.rueda_izda = izda;
		this.rueda_dcha = dcha;
	}
	
	/**
	 * Asigna un nuevo valor a cada uno de los encoders del motor.
	 * 
	 * @param izdo El valor a asignar al encoder izquierdo
	 * @param dcho El valor a asignar al encoder derecho
	 * 
	 */
	public void setEncoders(double izdo, double dcho){
		this.encoder_izdo = izdo;
		this.encoder_dcho = dcho;
	}
	
	/**
	 * Asigna un nuevo valor a la distancia recorrida por el robot.
	 * 
	 * @param dist El nuevo valor de la distancia
	 * 
	 */
	public void setDistancia(double dist){
		this.distancia = dist/10;
	}
	
	/**
	 * Asigna un nuevo valor al giro realizado por el robot.
	 * 
	 * @param giro El nuevo valor del giro
	 * 
	 */
	public void setGiro(double giro){
		this.giro = giro/10;
	}
	
	
}
