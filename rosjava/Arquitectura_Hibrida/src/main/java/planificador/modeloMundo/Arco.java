package planificador.modeloMundo;

/**
 * Clase que representa una conexiun entre dos localizaciones (nodos) 
 * del mapa de movimientos que representa el entorno del robot. Un arco 
 * define la estrategia de movimiento que permite llegar desde una de las 
 * localizaciones hasta la otra.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Arco {
	
	/**
	 * Uno de los nodos que forman la conexiun entre dos nodos del mapa.
	 */
	private Nodo localizacion1;
	
	/**
	 * Uno de los nodos que forman la conexiun entre dos nodos del mapa.
	 */
	private Nodo localizacion2;
	
	/**
	 * Estrategia de movimiento para alcanza uno de los nodos desde el otro, y viceversa.
	 */
	private String estrategia;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Arco(){}
	
	/**
	 * Inicializa el arco, asignando los dos nodos que lo forman y la estrategia de 
	 * movimiento entre ellos.
	 * 
	 * @param loc1 Uno de los nodos del arco
	 * @param loc2 Uno de los nodos del arco
	 * @param estr Estrategia de movimiento del arco
	 */
	public Arco(Nodo loc1, Nodo loc2, String estr){
		this.localizacion1 = loc1;
		this.localizacion2 = loc2;
		this.estrategia = estr;
	}
	
	
	/**
	 * Devuelve las dos localizaciones que forman el arco dentro del mapa.
	 * 
	 * @return El array que contiene los dos nodos del arco
	 */
	public Nodo[] getLocalizaciones(){
		Nodo[] localizaciones = new Nodo[2];
		localizaciones[0] = this.localizacion1;
		localizaciones[1] = this.localizacion2;
		return localizaciones;
	}
	
	/**
	 * Devuelve la estrategia de movimiento que permite ir de un nodo del mapa a 
	 * otro mediante el arco.
	 * 
	 * @return La estrategia de movimiento del arco
	 */
	public String getEstrategia(){
		return this.estrategia;
	}

}
