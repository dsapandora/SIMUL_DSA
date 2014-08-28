package controlador.gestorHabilidades;

import infraestructura.Robot;

/**
 * Clase que representa un metodo de ejecucion que el robot 
 * puede llevar a cabo para resolver tareas. Es una generalizacion 
 * de los diferentes tipos de habilidades que puede ejecutar el 
 * robot.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public abstract class Habilidad {
	
	/**
	 * Identificador unico de una habilidad, correspondiente con el nombre dado para esa habilidad.
	 */
	private String identificador;

	/**
	 * Referencia al robot del sistema, que hace uso de la habilidad.
	 */
	protected Robot robot;
	
	/**
	 * Indica si la habilidad se esta ejecutando sobre el simulador o sobre el robot real.
	 */
	private boolean sim;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Habilidad(){}
	
	/**
	 * Inicializa la habilidad del robot asignando un identificador concreto.
	 * 
	 * @param id El nombre de la habilidad
	 * @param robot El robot que hace uso de la habilidad
	 * @param sim A "true" si se utiliza la arquitectura sobre simulador
	 * 
	 */
	public Habilidad(String id, Robot robot, boolean sim){
		this.identificador = id;
		this.robot = robot;
		this.sim = sim;
	}
	
	
	/**
	 * Devuelve el identificador unico de la habilidad.
	 * 
	 * @return El nombre de la habilidad
	 * 
	 */
	public String getId(){
		return this.identificador;
	}
	
	/**
	 * Devuelve la referencia al robot que hace uso de la habilidad.
	 * 
	 * @return El robot del sistema
	 * 
	 */
	public Robot getRobot(){
		return this.robot;
	}
	
	/**
	 * Devuelve si la habilidad se esta ejecutando en simulador o en el robot real.
	 * 
	 * @return "true" si se esta ejecutando en simulador, "false" en caso contrario
	 */
	public boolean getSim(){
		return this.sim;
	}
	
	/**
	 * Asigna un identificador a la habilidad.
	 * 
	 * @param id El nuevo identificador
	 * 
	 */
	public void setID(String id){
		this.identificador = id;
	}
	
	/**
	 * Metodo de ejecucion concreto de la habilidad. Se trata del bucle infinito que contiene
	 * las instrucciones a ejecutar por el robot para que presente el comportamiento ligado a la
	 * habilidad dentro del modelo del mundo.
	 */
	public abstract void ejecutar();
	
	/**
	 * Metodo de comprobacion de la condicion de exito de una tarea. Se trata de una
	 * comprobacion unica y concreta del estado del modelo del mundo que indica si la ejecucion
	 * de la habilidad asociada a la tarea ha resultado satisfactoria, es decir, si se ha 
	 * alcanzado la submeta de la tarea para la cual se ha ejecutado una/s habilidad/es.
	 * 
	 * @return "true" si se ha alcanzado la submeta de la tarea
	 */
	public abstract boolean comprobarExito();
	
	
}
