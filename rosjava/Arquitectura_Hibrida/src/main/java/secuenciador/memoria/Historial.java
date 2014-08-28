package secuenciador.memoria;

import java.util.LinkedList;
import planificador.gestorPlanes.Plan;
import secuenciador.interprete.Tarea;

/**
 * Clase que almacena el historial de  planes pasados y sus 
 * soluciones para facilitar la secuenciaciun del plan actual, 
 * y proporciona acceso a ustos.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Historial {

	/**
	 * Lista de planes almacenados en el historial del sistema.
	 */
	private LinkedList<Plan> planes;
	
	/**
	 * Lista de las soluciones (RAPs) dadas a los planes almacenados en el historial.
	 */
	private LinkedList<LinkedList<RAP>> soluciones;
	
	/**
	 * Lista de las secuencias de tareas dadas a los planes almacenados en el historial.
	 */
	private LinkedList<LinkedList<Tarea>> secuencias;
	
	
	/**
	 * Inicializa por defecto. El historial se debe ir rellenando
	 * a medida que se ejecutan planes. Si se desea mantener el historial del sistema, habru
	 * que sustituir esta clase por una base de datos.
	 */
	public Historial(){
		this.planes = new LinkedList<Plan>();
		this.soluciones = new LinkedList<LinkedList<RAP>>();
		this.secuencias = new LinkedList<LinkedList<Tarea>>();
	}
	
	
	/**
	 * Devuelve la lista de planes almacenados en el historial.
	 * 
	 * @return La lista de todos los planes
	 */
	public LinkedList<Plan> getPlanes(){
		return this.planes;
	}
	
	/**
	 * Devuelve la lista de RAPs asignados a un plan concreto cuando uste se resolviu.
	 * 
	 * @param plan El plan del que se quiere obtener la soluciun
	 * @return La lista de RAPs asignados al plan
	 */
	public LinkedList<RAP> getSolucion(Plan plan){
		int pos = this.planes.indexOf(plan);
		return this.soluciones.get(pos);
	}
	
	/**
	 * Devuelve la secuencia de tareas generada para un plan concreto cuando uste se 
	 * resolviu.
	 * 
	 * @param plan El plan del que se quiere obtener la secuencia
	 * @return La secuencia de tareas realizada para el plan
	 */
	public LinkedList<Tarea> getSecuencia(Plan plan){
		int pos = this.planes.indexOf(plan);
		return this.secuencias.get(pos);
	}
	
	/**
	 * Almacena en el historial un plan llevado a cabo con uxito, junto con la secuencia 
	 * de tareas generada para uste y la lista de RAPs utilizados para llevarlo a cabo.
	 * 
	 * @param plan El plan que se desea almacenar
	 * @param solucion La lista de RAPs asignados al plan
	 * @param secuencia La secuencia de tareas generada para el plan
	 */
	public void guardarPlan(Plan plan, LinkedList<RAP> solucion, LinkedList<Tarea> secuencia){
		this.planes.add(plan);
		this.soluciones.add(solucion);
		this.secuencias.add(secuencia);
	}
	
	
}
