package infraestructura;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
/**
 * Clase que representa un elemento hardware genrico que 
 * permite al robot obtener informacin externa del mundo. 
 * Es una generalizacin de los diferentes tipos de sensores 
 * de los que puede hacer uso el robot.
 * 
 * @author Jos Luis Daz Cebrin
 * @version 1.0
 * 
 */
public abstract class Sensor {

	/**
	 * Identificador nico correspondiente con el nombre del sensor.
	 */
	private String identificador;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Sensor(){}
	
	/**
	 * Inicializa el sensor asignando un identificador concreto.
	 * 
	 * @param id El nombre del sensor
	 * 
	 */
	public Sensor(String id){
		this.identificador = id;
	}
	
	
	/**
	 * Devuelve el identificador nico del sensor.
	 * 
	 * @return El nombre del sensor
	 * 
	 */
	public String getId(){
		return this.identificador;
	}
	
	/**
	 * Asigna un identificador al sensor.
	 * 
	 * @param id El nuevo identificador
	 * 
	 */
	public void setId(String id){
		this.identificador = id;
	}
	
}
