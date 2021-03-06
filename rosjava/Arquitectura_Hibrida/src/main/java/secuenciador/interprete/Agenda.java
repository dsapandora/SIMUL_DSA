package secuenciador.interprete;

import java.util.LinkedList;

import planificador.gestorPlanes.Plan;

/**
 * Clase que trata los planes, transformando las acciones 
 * de ustos en tareas, y dundoles un orden. Las tareas 
 * pendientes se almacenan en una lista y se van sacando 
 * segun sea necesario.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Agenda {

	/**
	 * Plan a llevar a cabo por el robot, del que se obtienen la agenda de tareas.
	 */
	private Plan plan;
	
	/**
	 * Lista ordenada de tareas a llevar a cabo para realizar satisfacoriamente el plan.
	 */
	public LinkedList<Tarea> tareas;
	
	
	/**
	 * Inicializa la agenda por defecto con sus atributos a null, puesto que la agenda
	 * no puede rellenarse hasta que el planificador elabore el plan, y posteriormente
	 * se secuencie.
	 */
	public Agenda(){
		this.plan = null;
		this.tareas = new LinkedList<Tarea>();
	}
	
	
	/**
	 * Devuelve el plan actual asignado a la agenda.
	 * 
	 * @return El plan a llevar a cabo mediante la agenda
	 * 
	 */
	public Plan getPlan(){
		return this.plan;
	}
	
	/**
	 * Indica si existen o no tareas pendientes en la agenda.
	 * 
	 * @return "true" si hay tareas pendientes, "false" en caso contrario
	 * 
	 */
	public boolean hayMasTareas(){
		return !(this.tareas.isEmpty());
	}
	
	/**
	 * Devuelve la primera tarea almacenada en la agenda, segun el orden indicado
	 * por la lista de tareas de la agenda.
	 * 
	 * @return La primera tarea pendiente
	 * 
	 */
	public Tarea siguiente(){
		return this.tareas.pollFirst();
	}
	
	/**
	 * Almacena un nuevo plan en la agenda. Normalmente se registra un nuevo plan cuando
	 * el anterior ha fallado y ha sido necesaria una replanificaciun completa.
	 * 
	 * @param plan El nuevo plan a asignar a la agenda
	 * 
	 */
	public void registrarPlan(Plan plan){
		this.plan = plan;
	}
	
	/**
	 * Rellena los atributos de la agenda que no se pudieron inicializar al crear la
	 * instancia de la agenda.
	 * 
	 * @param plan El plan elaborado por el planificador para llevar a cabo
	 * @param secuencia La lista ordenada de tareas creada a partir del plan
	 */
	public void crearAgenda(Plan plan, LinkedList<Tarea> secuencia){
		this.plan = plan;
		int i = 0;
		while(i<secuencia.size()){
			this.tareas.add(secuencia.get(i));
			i++;
		}		
	}
	
	/**
	 * Auade una nueva tarea en la agenda en una posiciun determinada.
	 * 
	 * @param tarea La nueva tarea a llevar a cabo
	 * @param posicion La posiciun dentro de la lista de tareas
	 */
	public void anyadirTarea(Tarea tarea, int posicion){
		this.tareas.add(posicion, tarea);
	}
	
	/**
	 * Elimina una tarea de la agenda.
	 * 
	 * @param tarea La tarea a eliminar de la lista de tareas
	 */
	public void eliminarTarea(Tarea tarea){
		this.tareas.remove(tarea);
	}
	
	
}
