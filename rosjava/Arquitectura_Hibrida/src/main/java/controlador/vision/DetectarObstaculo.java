package controlador.vision;

import infraestructura.Robot;
import controlador.gestorHabilidades.Habilidad;

/**
 * Clase abstracta que hereda de Habilidad y define las caracteristicas 
 * propias de esta habilidad e implementa su bucle de ejecucion 
 * y la condicion del evento de finalizacion. Sirve como ejemplo de 
 * extensibilidad de la funcionalidad del robot a la hora de anadir 
 * nuevas habilidades que puede ejecutar.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public abstract class DetectarObstaculo extends Habilidad{

	/**
	 * TODO Incluir los atributos necesarios para ejecutar la nueva habilidad.
	 */
	
	
	/**
	 * TODO Completar el codigo del constructor de la habilidad.
	 */
	public DetectarObstaculo(Robot robot, boolean sim){
		super("DetectarObstaculo", robot, sim);
	}
		
	/**
	 * @see Habilidad#ejecutar()
	 */
	public abstract void ejecutar();
	
	/**
	 * @see Habilidad#comprobarExito()
	 */
	public abstract boolean comprobarExito();
	
}
