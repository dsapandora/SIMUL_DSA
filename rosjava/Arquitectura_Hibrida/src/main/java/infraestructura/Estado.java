package infraestructura;

import planificador.modeloMundo.Nodo;
import secuenciador.interprete.Tarea;

/**
 * Clase que representa el estado del robot dentro del modelo 
 * del mundo: su posici�n, la tarea actual encomendada, el nivel 
 * de bater�a y dem�s elementos significativos definidos para el 
 * problema. 
 * 
 * @author Jos� Luis D�az Cebri�n
 * @version 1.0
 * 
 */
public class Estado {
	
	/**
	 * La posici�n del robot dentro del mapa en cada momento.
	 */
	private Nodo posicion;
	
	/**
	 * La tarea que est� llevando a cabo el robot en el momento actual.
	 */
	private Tarea tarea_actual;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Estado(){}
	
	/**
	 * Inicializa el estado del robot asignando una posici�n y tarea iniciales.
	 * 
	 * @param pos La posici�n inicial del robot
	 * @param tarea La tarea inicial que debe llevar a cabo el robot
	 * 
	 */
	public Estado(Nodo pos, Tarea tarea){
		this.posicion = pos;
		this.tarea_actual = tarea;
	}
	
	
	/**
	 * Devuelve la posici�n actual del robot dentro del mapa.
	 * 
	 * @return La posici�n actual del robot
	 * 
	 */
	public Nodo getPosicion(){
		return this.posicion;
	}
	
	/**
	 * Devuelve la tarea actual que est� llevando a cabo el robot.
	 * 
	 * @return La tarea actual del robot
	 * 
	 */
	public Tarea getTarea(){
		return this.tarea_actual;
	}
	
	/**
	 * Asigna una nueva posici�n al robot dentro del mapa.
	 * 
	 * @param pos La nueva posici�n del robot en el mapa
	 * 
	 */
	public void setPosicion(Nodo pos){
		this.posicion = pos;
	}
	
	/**
	 * Asigna una nueva tarea a llevar a cabo por el robot.
	 * 
	 * @param tarea La nueva tarea a asignar al robot
	 * 
	 */
	public void setTarea(Tarea tarea){
		this.tarea_actual = tarea;
	}

}
