package planificador.modeloMundo;

import java.util.LinkedList;

/**
 * Clase que representa el mapa del modelo mundo, por el que el robot 
 * se mueve. Se trata de un mapa de coordenadas, en el que se representan 
 * todas las posibles posiciones del robot y los objetos en el entorno de 
 * forma continuar. Sirve como apoyo a la planificaciun para la identificaciun 
 * de submetas en busca de la meta global del robot. Contiene la informaciun 
 * sobre los elementos relevantes identificados para el problema, tales como la 
 * posiciun del robot, de la meta o de la posible base de recarga de la baterua.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 *
 */
public abstract class Coordenadas extends Mapa{

	/**
	 * Longitud del eje X (abscisas) del mapa.
	 */
	private int long_x;
	
	/**
	 * Longitud del eje Y (ordenadas) del mapa.
	 */
	private int long_y;
	
	/**
	 * Lista de todas las posiciones relevantes del mapa.
	 */
	private LinkedList<Posicion> posiciones;
	
	
	/**
	 * Inicializa el mapa del mundo a partir del fichero de configuraciun.
	 * 
	 * @param fich El nombre del fichero de configuraciun que describe el mapa
	 */
	public Coordenadas(String fich){
		super(fich);
		/* TODO Construir el mapa de coordenadas leyendo el fichero */
	}
	
	
	/**
	 * Devuelve la longitud del eje de abscisas del mapa.
	 * 
	 * @return La longitud del eje X
	 */
	public int getLongitudX(){
		return this.long_x;
	}
	
	/**
	 * Devuelve la longitd del eje de ordenadas del mapa.
	 * 
	 * @return La longitud del eje Y
	 */
	public int getLongitudY(){
		return this.long_y;
	}
	
	/**
	 * Auade una nueva posiciun relevante a la lista de posiciones del mapa.
	 * 
	 * @param posicion La posiciun a auadir a la lista
	 */
	public void anyadirNodo(Posicion posicion){
		this.posiciones.add(posicion);
	}
	
}
