package planificador.modeloMundo;

import utilidades.FicheroLectura;

/**
 * Clase que representa el mapa del modelo mundo, por el que el robot 
 * se mueve. Se trata de una superclase que permite trabajar con distintos
 * tipos de mapas, como mapas de coordenadas o de movimientos.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 *
 */
public abstract class Mapa {

	/**
	 * Nombre del fichero de configuraciun del mapa.
	 */
	private String nombre_fichero;
	
	/**
	 * Fichero de configuraciun del mapa a partir del cual se crea la 
	 * instancia del mapa.
	 */
	private FicheroLectura fichero;
	
	
	/**
	 * Inicializa el fichero de configuraciun para su posterior lectura por la 
	 * subclase adecuada.
	 * 
	 * @param fich El nombre del fichero de configuraciun del mapa
	 */
	public Mapa(String fich){
		this.fichero = new FicheroLectura(fich);
	}
	
	
	/**
	 * Devuelve el nombre del fichero de configuraciun del mapa.
	 * 
	 * @return El nombre del fichero
	 */
	public String getNombre(){
		return this.nombre_fichero;
	}
	
	/**
	 * Devuelve la instancia del fichero de configuraciun del mapa.
	 * 
	 * @return El fichero del mapa
	 */
	public FicheroLectura getFichero(){
		return this.fichero;
	}
	
}
