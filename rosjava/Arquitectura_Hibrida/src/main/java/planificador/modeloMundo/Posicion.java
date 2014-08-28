package planificador.modeloMundo;

/**
 * Clase que representa una posiciun dentro de un mapa de coordenadas 
 * segun sus valores en el eje de abscisas y ordenadas.
 * 
 * @author Josu Luis Diaz Cebriun
 * @version 1.0
 *
 */
public class Posicion {

	/**
	 * Valor de la coordenada X (eje de abscisas) en el mapa.
	 */
	private double coord_x;
	
	/**
	 * Valor de la coordenada Y (eje de ordenadas) en el mapa.
	 */
	private double coord_y;
	
	
	/**
	 * Inicializa por defecto con los atributos a 0.
	 */
	public Posicion(){}
	
	/**
	 * Inicializa una posiciun en el mapa con unos valores concretos para las 
	 * coordenadas.
	 * 
	 * @param x El valor de la coordenada X
	 * @param y El valor de la coordenada Y
	 */
	public Posicion(double x, double y){
		this.coord_x = x;
		this.coord_y = y;
	}
	
	
	/**
	 * Devuelve el valor de la coordenada X de esta posiciun.
	 * 
	 * @return La coordenada X de la posiciun
	 */
	public double getX(){
		return this.coord_x;
	}
	
	/**
	 * Devuelve el valor de la coordenada Y de esta posiciun.
	 * 
	 * @return La coordenada Y de la posiciun
	 */
	public double getY(){
		return this.coord_y;
	}
	
	/**
	 * Asigna un nuevo valor a la coordenada X para esta posiciun.
	 * 
	 * @param x El valor a asignar a la coordenada X
	 */
	public void setX(double x){
		this.coord_x = x;
	}
	
	/**
	 * Asigna un nuevo valor a la coordenada Y para esta posiciun.
	 * 
	 * @param y El valor a asignar a la coordenada Y
	 */
	public void setY(double y){
		this.coord_y = y;
	}
	
}
