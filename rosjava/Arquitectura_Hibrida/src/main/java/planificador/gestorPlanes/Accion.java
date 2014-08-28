package planificador.gestorPlanes;

import java.util.LinkedList;

/**
 * Clase que representa una operacion del concepto clasico de 
 * planificacion. A partir de unos predicados que indican las 
 * precondiciones, ejecutando esta accion se llega a cambiar el 
 * modelo del mundo con otros predicados que indican las 
 * postcondiciones.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Accion {

	/**
	 * Identificador unico de la accion dentro del dominio del planificador.
	 */
	private String nombre;
	
	/**
	 * Serie de elementos que, junto con el nombre, definen la accion.
	 */
	private Elemento[] elementos;
	
	/**
	 * Lista de predicados que definen las precondiciones de la accion para que 
	 * pueda llevarse a cabo.
	 */
	private LinkedList<Predicado> precondiciones;
	
	/**
	 * Lista de predicados que es preciso borrar del estado del modelo del mundo 
	 * tras realizar la accion.
	 */
	private LinkedList<Predicado> borrados;
	
	/**
	 * Lista de predicados que es necesario incluir en el estado del modelo del 
	 * mundo tras realizar la accion.
	 */
	private LinkedList<Predicado> anyadidos;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Accion(){}
	
	/**
	 * Constructor por defecto para la libreria de acciones. NO USAR PARA CREAR 
	 * ACCIONES PARA EL PLANIFICADOR.
	 */
	public Accion(String nom){
		this.nombre = nom;
	}
	
	/**
	 * Inicializa la accion, asignandole un nombre y los elementos que la definen. 
	 * Posteriormente se debe asignar la lista de precondiciones y postcondiciones.
	 * 
	 * @param nom El identificador (nombre) de la accion
	 * @param elem El array de elementos que definen la accion
	 */
	public Accion(String nom, Elemento[] elem){
		this.nombre = nom;
		this.elementos = new Elemento[elem.length];
		for(int i=0; i<this.elementos.length; i++)
			this.elementos[i] = elem[i];
		this.precondiciones = new LinkedList<Predicado>();
		this.borrados = new LinkedList<Predicado>();
		this.anyadidos = new LinkedList<Predicado>();
	}
	
	
	/**
	 * Devuelve el identificador unico (nombre) de la accion.
	 * 
	 * @return El nombre de la accion
	 */
	public String getNombre() {
		return this.nombre;
	}
	
	/**
	 * Devuelve la serie de elementos que definen la accion.
	 * 
	 * @return El array de elementos de la accion
	 */
	public Elemento[] getElementos(){
		return this.elementos;
	}
	
	/**
	 * Devuelve la lista de predicados que forman las precondiciones de la accion.
	 * 
	 * @return La lista de precondiciones
	 */
	public LinkedList<Predicado> getPrecondiciones(){
		return this.precondiciones;
	}
	
	/**
	 * Devuelve la lista de predicados que forman las postcondiciones de borrado 
	 * de la accion.
	 * 
	 * @return La lista de borrados
	 */
	public LinkedList<Predicado> getBorrados(){
		return this.borrados;
	}
	
	/**
	 * Devuelve la lista de predicados que forman las postcondiciones de inclusion 
	 * de la accion.
	 * 
	 * @return La lista de anadidos
	 */
	public LinkedList<Predicado> getAnyadidos(){
		return this.anyadidos;
	}
	
	/**
	 * Asigna un nuevo identificador (nombre) a la accion.
	 * 
	 * @param nombre El nuevo nombre de la accion
	 */
	public void SetNombre(String nombre){
		this.nombre = nombre;
	}
	
	/**
	 * Asigna una lista de elementos a la definicion de la accion.
	 * 
	 * @param elem El array de elementos que definen la accion
	 */
	public void setElementos(Elemento[] elem){
		this.elementos = new Elemento[elem.length];
		for(int i=0; i<this.elementos.length; i++)
			this.elementos[i] = elem[i];
	}
	
	/**
	 * Incluye un nuevo predicado a la lista de precondiciones de la accion.
	 * 
	 * @param pred El predicado a anadir a la lista
	 */
	public void anyadirPrecondicion(Predicado pred){
		this.precondiciones.add(pred);
	}
	
	/**
	 * Incluye un nuevo predicaado a la lista de postcondiciones de borrado 
	 * de la accion.
	 * 
	 * @param pred El predicado a anadir a la lista
	 */
	public void anyadirBorrado(Predicado pred){
		this.borrados.add(pred);
	}
	
	/**
	 * Incluye un nuevo predicado a la lista de postcondiciones de inclusion 
	 * de la accion.
	 * 
	 * @param pred El predicado a anadir a la lista
	 */
	public void anyadirAnyadido(Predicado pred){
		this.anyadidos.add(pred);
	}

}
