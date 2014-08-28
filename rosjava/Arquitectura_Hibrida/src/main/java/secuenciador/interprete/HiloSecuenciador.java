package secuenciador.interprete;

import infraestructura.Robot;
import java.io.IOException;
import java.io.PipedReader;
import java.io.PipedWriter;
import java.util.LinkedList;
import java.util.concurrent.Semaphore;

import controlador.gestorHabilidades.HiloControlador;

import planificador.gestorPlanes.Meta;
import planificador.gestorPlanes.Plan;
import secuenciador.memoria.Monitor;

/**
 * Thread que mantiene la ejecuciun paralela del subsistema 
 * Secuenciador, ejecutando la parte capa intermedia de la 
 * arquitectura (secuenciaciun). Gestiona las comunicaciones 
 * con el resto de hilos de ejecuciun.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class HiloSecuenciador extends Thread{
	
	/**
	 * Prioridad por defecto del hilo controlador.
	 */
	private final int PRIORIDAD;
	
	/**
	 * Semuforo de un permiso (mutex) para el control concurrente.
	 */
	private Semaphore mutex;
	
	/**
	 * Referencia al secuenciador del sistema.
	 */
	private HiloControlador controlador;
	
	/**
	 * Referencia al objeto que se encarga de secuenciar en tareas el plan elaborado 
	 * previamente por el planificador.
	 */
	private Secuenciacion secuenciacion;
	
	/**
	 * Referencia al objeto que se encarga de monitorizar el estado del robot y 
	 * actualizarlo para tener en todo momento informaciun actual del mismo.
	 */
	private Monitor monitor;
	
	/**
	 * Plan a secuenciar para poder ser llevado a cabo por el robot del sistema.
	 */
	private Plan plan;
	
	/**
	 * Agenda que contiene la secuencia de tareas pendientes ordenadas y que permite
	 * su modificaciun en tiempo real para reaccionar a las situaciones imprevistas.
	 */
	private Agenda agenda;
	
	/**
	 * Tarea actual que debe llevar a cabo el robot antes de seguir con una nueva tarea 
	 * de la agenda.
	 */
	private Tarea tarea_actual;
	
	/**
	 * Flujo de comunicaciun (escritura) con el hilo controlador.
	 */
	private PipedWriter com_controlador;
	
	/**
	 * Flujo de comunicaciun (lectura) con el hilo planificador.
	 */
	private PipedReader com_planificador;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public HiloSecuenciador(){
		this.PRIORIDAD = 4;
	}
	
	/**
	 * Inicializa el secuenciador y sus comunicaciones, crea una instancia de secuenciaciun 
	 * y crea una agenda vacua, en espera del plan elaborado para poder secuenciar a 
	 * continuaciun.
	 * 
	 * @param robot El robot del sistema que utiliza la arquitectura
	 * @param com_c El flujo de lectura por el que recibe informaciun el controlador
	 * @param com_p El flujo de escritura por el que manda informaciun el planificador
	 */
	public HiloSecuenciador(Robot robot, PipedReader com_c, PipedWriter com_p){
		this.PRIORIDAD = 4;
		this.setPriority(this.PRIORIDAD);
		this.secuenciacion = new Secuenciacion();
		this.monitor = new Monitor(robot);
		this.plan = null;
		this.agenda = new Agenda();
		this.tarea_actual = new Tarea("Inicio", null);
		/* TODO Descomentar para utilizar ejecuciun concurrente */
		/*try {
			this.com_controlador = new PipedWriter(com_c);
			this.com_planificador = new PipedReader(com_p, 1024);
		} catch (IOException e) {
			System.out.println("ERROR al crear los flujos de comunicaciun con los hilos controlador y planificador");
		}*/
	}
	
	
	/**
	 * Devuelve la agenda, con la lista ordenada de tareas que debe llevar a cabo el robot.
	 * 
	 * @return La agenda de tareas
	 */
	public Agenda getAgenda(){
		return this.agenda;
	}
	
	/**
	 * Devuelve la tarea actual que estu llevando a cabo el robot.
	 * 
	 * @return La tarea actual
	 */
	public Tarea getTareaActual(){
		return this.tarea_actual;
	}
	
	/**
	 * Devuelve el objeto que permite realizar la secuenciaciun del plan.
	 * 
	 * @return El objeto de secuenciaciun
	 */
	public Secuenciacion getSecuenciacion(){
		return this.secuenciacion;
	}
	
	/**
	 * Devuelve el monitor del sistema.
	 * 
	 * @return El monitor del sistema
	 */
	public Monitor getMonitor(){
		return this.monitor;
	}
	
	/**
	 * Devuelve el plan que estu llevando a cabo el robot.
	 * 
	 * @return El plan elaborado para el robot
	 */
	public Plan getPlan(){
		return this.plan;
	}
	
	/**
	 * Asigna un plan una vez que ha sido elaborado. Ademus, este mutodo se encarga de pasar 
	 * este plan a todas las instancias que lo necesiten.
	 * 
	 * @param plan El nuevo plan a ejecutar por el robot
	 */
	public void setPlan(Plan plan){
		this.plan = plan;
		this.secuenciacion.actualizarPlan(plan);
		this.agenda.registrarPlan(plan);
	}
	
	/**
	 * Asigna una nueva tarea para que el robot lleve a cabo desde este momento.
	 * 
	 * @param tar La nueva tareas a asignar al robot
	 */
	public void setTareaActual(Tarea tar){
		this.tarea_actual = tar;
	}
	
	/**
	 * Realiza una modificaciun de la agenda, ya sea auadiendo o eliminando una tarea
	 * de una posiciun concreta de la lista de tareas pendientes.
	 * 
	 * @param accion "add" auade una tarea nueva; "rem" elimina una tarea existente
	 * @param tar La tarea sobre la que se realiza la acciun
	 * @param pos La posiciun en la agenda en caso de que se quiera auadir la tarea
	 */
	public void modificarAgenda(String accion, Tarea tar, int pos){
		if(accion.compareTo("add")==0)
			this.agenda.anyadirTarea(tar, pos);
		else if(accion.compareTo("rem")==0)
			this.agenda.eliminarTarea(tar);
		else
			System.out.println("Error al modificar agenda: Operaciun " + accion + " no definida.");
	}
	
	/**
	 * Mutodo que realiza el conjunto de acciones que permite dar un nuevo paso en la 
	 * secuencia de tareas, es decir, cuando una tarea se lleva a cabo con uxito, toma la 
	 * siguiente de la agenda, se lo comunica al hilo controlador y se selecciona una nueva 
	 * habilidad en funciun del RAP asignado a la nueva tarea.
	 * 
	 * @param pos Posiciun de la nueva tarea dentro de la secuencia de tareas
	 */
	public void nuevoPasoSecuencia(int pos){
		this.tarea_actual = this.agenda.siguiente();
		this.controlador.cambiarTarea(this.tarea_actual);
		LinkedList<String> habs = this.secuenciacion.getSolucion().get(pos).getHabilidades();
		this.controlador.activarConjunto(habs);
		String hab = this.secuenciacion.seleccionarHabilidad(this.secuenciacion.getSolucion().get(pos), this.monitor.getEstado(), this.monitor.getPosicion());
		this.controlador.setHabilidadActual(hab);
	}
	
	/**
	 * Mutodo que realiza las operaciones necesarias para cambiar la secuencia actual de 
	 * tareas tras detectar una colisiun durante la ejecuciun de la tarea actual.
	 */
	public void informarChoque(){
		this.getAgenda().anyadirTarea(this.tarea_actual, 0);
		this.getAgenda().anyadirTarea(new Tarea("Retroceder", new Meta("SuperarChoque", null)), 0);
		this.recuperarSecuencia();
	}
	
	/**
	 * Mutodo que se encarga de modificar la secuencia de tareas tras ser informado de 
	 * un fallo en la ejecuciun de la tarea actual. Coloca en el primer lugar de la agenda 
	 * una tarea de recuperaciun y actualiza el conjunto de habilidades y la habilidad actual
	 * para llevar a cabo la recuperaciun del robot.
	 */
	public void recuperarSecuencia(){
		this.controlador.cambiarTarea(this.getAgenda().siguiente());
		LinkedList<String> habs = new LinkedList<String>();
		habs.add("Recuperarse");
		this.controlador.activarConjunto(habs);
		this.controlador.setHabilidadActual("Recuperarse");
	}
	
	/**
	 * Lee todos los caracteres que se encuentren en el buffer del flujo de comunicaciun
	 * con el hilo planificador.
	 * 
	 * @return La cadena de caracteres leudos
	 * 
	 */
	public String leerMensaje(){
		String mensaje = "";
		char c;
		try{
			while(this.com_planificador.ready()){
				c = (char) this.com_planificador.read();
				mensaje = mensaje + c;
			}
		} catch(IOException e){
			System.out.println("ERROR al leer del comunicador con el hilo planificador.");
		}
		return mensaje;
	}
	
	/**
	 * Escribe una serie de caracteres en el buffer del flujo de comunicaciun
	 * con el hilo controlador.
	 * 
	 * @param mensaje La cadena de caracteres a escribir
	 * 
	 */
	public void escribirMensaje(String mensaje){
		try{
			this.com_controlador.write(mensaje);
		} catch(IOException e){
			System.out.println("ERROR al escribir en el comunicador con el hilo controlador.");
		}
	}
	
	/**
	 * Mutodo de ejecuciun del hilo secuenciador, heredado de la clase Thread.
	 * 
	 * TODO Este mutodo sirve para la ejecuciun secuencial. Para usar la ejecuciun concurrente, comentar este mutodo y descomentar el siguiente
	 */
	public void run(){
		
		/* Bucle hasta que se tiene un plan */
		while(this.plan==null){
			System.out.println("Esperando planificaciun...");
		}
		
		/* Se secuencia el plan y se crea la agenda de tareas */
		this.secuenciacion.actualizarPlan(this.plan);
		this.secuenciacion.secuenciar();
		this.agenda.crearAgenda(this.plan, this.secuenciacion.getSecuencia());
		
		/* Se actualiza la informaciun del robot mediante el monitor */
		this.monitor.actualizar();
		
		/* Se inicializa el controlador con la primera tarea y habilidad */
		int posicion_tarea = 0;
		this.nuevoPasoSecuencia(posicion_tarea);
		
		/* Bucle hasta que se ejecuta toda la secuencia de tareas */
		while(this.agenda.hayMasTareas()){			
			/* Se actualiza la informaciun del robot mediante el monitor */
			this.monitor.actualizar();
			
		}
		
		/* El plan se ha llevado a cabo con uxito, por lo que se registra la soluciun */
		this.secuenciacion.getHistorial().guardarPlan(this.plan, this.secuenciacion.getSolucion(), this.secuenciacion.getSecuencia());
		
	}
	
	/**
	 * Mutodo de ejecuciun del hilo secuenciador, heredado de la clase Thread.
	 * 
	 * TODO Este mutodo sirve para la ejecuciun concurrente. Para usar la ejecuciun concurrente, comentar este mutodo y descomentar el siguiente
	 */
	/*public void run(){
		
		// Bucle hasta que se tiene un plan
		while(this.plan==null){
			System.out.println("Esperando planificaciun...");
		}
		
		// Se secuencia el plan y se crea la agenda de tareas
		this.secuenciacion.actualizarPlan(this.plan);
		this.secuenciacion.secuenciar();
		this.agenda.crearAgenda(this.plan, this.secuenciacion.getSecuencia());
		
		// Se actualiza la informaciun del robot mediante el monitor
		this.monitor.actualizar();
		
		// Se inicializa el controlador con la primera tarea y habilidad
		int posicion_tarea = 0;
		this.nuevoPasoSecuencia(posicion_tarea);
		
		try {
			sleep(500);
		} catch (InterruptedException e) {
			System.out.println("ERROR del secuenciador mientras dormua");
		}
		
		// Bucle hasta que se ejecuta toda la secuencia de tareas
		while(this.agenda.hayMasTareas()){			
			// Si se ha completado con uxito la tarea, sigue la secuencia
			try {
				this.mutex.acquire();
				if(this.controlador.getExito()){
					posicion_tarea++;
					this.nuevoPasoSecuencia(posicion_tarea);
				}
			} catch (InterruptedException e1) {
				System.out.println("ERROR del mutex al hacer lock");
			}
			this.mutex.release();
			
			// Se actualiza la informaciun del robot mediante el monitor
			this.monitor.actualizar();
			
		}
		
		// El plan se ha llevado a cabo con uxito, por lo que se registra la soluciun
		this.secuenciacion.getHistorial().guardarPlan(this.plan, this.secuenciacion.getSolucion(), this.secuenciacion.getSecuencia());
		
		// Se indica al controlador el final de la ejecuciun
		//this.controlador.cambiarTarea(new Tarea("Fin", null));
		
	}*/
	
	
}
