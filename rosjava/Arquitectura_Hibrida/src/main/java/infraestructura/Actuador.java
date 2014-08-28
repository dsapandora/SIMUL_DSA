package infraestructura;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * Clase que representa un elemento hardware genrico 
 * que permite al robot interactuar con el mundo. Es 
 * una generalizacin de los diferentes tipos de actuadores 
 * de los que puede hacer uso el robot.
 * 
 * @author Jos Luis Daz Cebrin
 * @version 1.0
 * 
 */
public abstract class Actuador {

	/**
	 * Identificador nico correspondiente con el nombre del actuador.
	 */
	private String identificador;
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Actuador(final ConnectedNode node){}
	
	/**
	 * Inicializa el actuador asignando un identificador concreto.
	 * 
	 * @param id El nombre del actuador
	 * 
	 */
	public Actuador(String id,final ConnectedNode node){
		this.identificador = id;
	}
	
	/**
	 * Devuelve el identificador nico del actuador.
	 * 
	 * @return El nombre del actuador
	 * 
	 */
	public String getId(){
		return this.identificador;
	}
	
	/**
	 * Asigna un identificador al actuador.
	 * 
	 * @param id El nuevo identificador
	 * 
	 */
	public void setId(String id){
		this.identificador = id;
	}
	
}
