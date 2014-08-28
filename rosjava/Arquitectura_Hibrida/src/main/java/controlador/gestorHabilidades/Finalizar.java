package controlador.gestorHabilidades;

import secuenciador.interprete.Tarea;
import infraestructura.Estado;
import infraestructura.Robot;

/**
 * Clase que hereda de Habilidad y define las caracteristicas 
 * propias de esta habilidad especial, que permite la desconexion
 * segura del robot cuando finaliza la ejecucion del plan.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Finalizar extends Habilidad{

	/**
	 * Inicializa la habilidad para su posterior ejecucion.
	 * 
	 * @param robot El robot del sistema que hace uso de la habilidad
	 * 
	 */
	public Finalizar(Robot robot, boolean sim){
		super("Finalizar", robot, sim);
	}
	
	/**
	 * @see Habilidad#ejecutar()
	 */
	public void ejecutar(){
		Estado estado = new Estado(null, new Tarea("Fin", null));
		super.robot.setEstado(estado);
	}
	
	/**
	 * @see Habilidad#comprobarExito()
	 */
	public boolean comprobarExito(){
		return true;
	}
}
