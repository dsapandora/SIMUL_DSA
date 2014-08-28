package planificador.modeloMundo;

import java.util.LinkedList;

/**
 * Clase que representa una localizaciun concreta dentro del mapa, 
 * definida por un nombre unico del nodo, y las localizaciones que 
 * estun conectadas a ellas mediante arcos.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Nodo {

	/**
	 * Identificador unico del nodo del mapa.
	 */
	private String identificador;
	
	/**
	 * Lista de conexiones del nodo con otros nodos del mapa.
	 */
	private LinkedList<Arco> conexiones;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Nodo(){}
	
	/**
	 * Inicializa el nodo asignundole un identificador unico. Posteriormente, se 
	 * deben asignar las conexiones de este nodo con otros nodos del mapa.
	 * 
	 * @param id El identificador del nodo
	 */
	public Nodo(String id){
		this.identificador = id;
		this.conexiones = new LinkedList<Arco>();
	}
	
	
	/**
	 * Devuelve el identificador unico del nodo.
	 * 
	 * @return El identificador del nodo
	 */
	public String getId(){
		return this.identificador;
	}
	
	/**
	 * Devuelve la lista de conexiones del nodo con otros nodos del mapa.
	 * 
	 * @return La lista de arcos en los que participa el nodo
	 */
	public LinkedList<Arco> getConexiones(){
		return this.conexiones;
	}
	
	/**
	 * Asigna un nuevo identificador unico al nodo.
	 * 
	 * @param id El nuevo identificador del nodo
	 */
	public void setId(String id){
		this.identificador = id;
	}
	
	/**
	 * Auade un nuevo arco a la lista de conexiones del nodo.
	 * 
	 * @param arco La conexiun a auadir a la lista
	 */
	public void anyadirConexion(Arco arco){
		this.conexiones.add(arco);
	}
	
	/**
	 * Indica si un nodo estu conectado con otro en el mapa del modelo del mundo. Dos
	 * nodos estun conectados si existe un arco en el mapa definido por estos dos nodos.
	 * 
	 * @param otro El nodo del que se desea saber su conexiun
	 * @return "true" si los dos nodos estun conectados, "false" en caso contrario
	 */
	public boolean estaConectado(Nodo otro){
		boolean conexion = false;
		int i = 0;
		while(i<this.conexiones.size() && !conexion){
			Nodo[] conx = this.conexiones.get(i).getLocalizaciones();
			if(conx[0].getId().compareTo(otro.getId())==0 || conx[1].getId().compareTo(otro.getId())==0)
				conexion = true;
		}
		return conexion;
	}
}
