package planificador.gestorPlanes;

/**
 * Clase que representa cualquier elemento del modelo del mundo 
 * con relevancia en la planificacion. Forma parte de los predicados 
 * para definir el estado del modelo del mundo en un momento concreto.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Elemento {

	/** 
	 * Tipo del elemento dentro del dominio del planificador.
	 */
	private String tipo;
	
	/**
	 * Identificador unico del elemento dentro del estado concreto del modelo del mundo.
	 */
	private String nombre;
	
	
	/**
	 * Inicializa por defectos con los atributos a null.
	 */
	public Elemento(){}
	
	/**
	 * Inicializa un elemento, asignando un tipo y un nombre (identificador unico).
	 * 
	 * @param tipo El tipo del elemento.
	 * @param nombre El identificador del elemento.
	 */
	public Elemento(String tipo, String nombre){
		this.tipo = tipo;
		this.nombre = nombre;
	}
	
	
	/**
	 * Devuelve el tipo del elemento.
	 * 
	 * @return El tipo del elemento
	 */
	public String getTipo() {
		return this.tipo;
	}
	
	/**
	 * Devuelve el identificador unico del elemento.
	 * 
	 * @return El nombre del elemento
	 */
	public String getNombre(){
		return this.nombre;
	}
	
	/**
	 * Asigna un nuevo tipo al elemento.
	 * 
	 * @param tipo El nuevo tipo del elemento
	 */
	public void setTipo(String tipo){
		this.tipo = tipo;
	}
	
	/**
	 * Asigna un nuevo identificador al elemento.
	 * 
	 * @param nombre El nuevo nombre del elemento
	 */
	public void setNombre(String nombre){
		this.nombre = nombre;
	}
	
	/**
	 * Indica si un elemento es igual a otro. Dos elementos son iguales si son del mismo 
	 * tipo y poseen el mismo identificador (nombre)
	 * 
	 * @param otro El elemento con el que se desea comparar
	 * @return "true" si los dos elementos son iguales, "false" en caso contrario
	 */
	public boolean esIgual(Elemento otro){
		boolean igual = false;
		if(this.tipo.compareTo(otro.getTipo())==0 && this.nombre.compareTo(otro.getNombre())==0)
			igual = true;
		return igual;
	}

}
