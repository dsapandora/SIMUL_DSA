package secuenciador.memoria;

import planificador.modeloMundo.Nodo;
import infraestructura.Estado;
import infraestructura.Robot;

/**
 * Clase que monitoriza el estado del robot y el modelo 
 * del mundo, registrando los cambios que se producen en 
 * ambos elementos. Se encarga tambiun de notificar al hilo 
 * de ejecuciun del secuenciador si se requiere de una 
 * replanificaciun debido a alguno de esos cambios.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Monitor {
	
	/**
	 * Referencia al robot del sistema, cuyo estado seru monitorizado.
	 */
	private Robot robot;
	
	/**
	 * Estado actual del robot.
	 */
	private Estado estado;
	
	/**
	 * Posiciun actual del robot dentro del mapa del mundo.
	 */
	private Nodo posicion_robot;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Monitor(){}
	
	/**
	 * Inicializa el monitor, asignando a uste la referencia al robot del sistema y
	 * actualizando por primera vez su estado.
	 * 
	 * @param robot El robot a monitorizar
	 */
	public Monitor(Robot robot){
		this.robot = robot;
		this.estado = robot.getEstado();
		this.posicion_robot = this.estado.getPosicion();
	}
	
	/**
	 * Devuelve el ultimo estado del robot monitorizado.
	 * 
	 * @return El estado del robot
	 */
	public Estado getEstado(){
		return this.estado;
	}
	
	/**
	 * Devuelve la ultima posiciun dentro del mapa en la que se encontraba el robot.
	 * 
	 * @return La posiciun del robot
	 */
	public Nodo getPosicion(){
		return this.posicion_robot;
	}
	
	/**
	 * Actualiza el estado almacenado con el estado actual real del robot.
	 */
	public void actualizarEstado(){
		this.estado = this.robot.getEstado();
	}

	/**
	 * Actualiza la posiciun almacenada con la posiciun actual real del robotl.
	 */
	public void actualizarPosicion(){
		this.posicion_robot = this.robot.getEstado().getPosicion();
	}
	
	/**
	 * Mutodo auxiliar que sirve para realizar una unica llamada de actualizaciun cuando
	 * es necesario monitorizar el estado y la posiciun actual del robot.
	 */
	public void actualizar(){
		this.actualizarEstado();
		this.actualizarPosicion();
	}
}
