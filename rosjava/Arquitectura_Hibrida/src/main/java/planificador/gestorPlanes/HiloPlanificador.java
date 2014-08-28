package planificador.gestorPlanes;

import infraestructura.Robot;
import java.io.IOException;
import java.io.PipedReader;
import java.io.PipedWriter;
import java.util.LinkedList;
import java.util.concurrent.Semaphore;

import planificador.modeloMundo.Mapa;
import planificador.modeloMundo.Movimientos;

/**
 * Thread que mantiene la ejecucion paralela del subsistema Planificador, 
 * ejecutando la parte capa deliberativa de la arquitectura (planificacion). 
 * Gestiona las comunicaciones con el resto de hilos de ejecucion.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class HiloPlanificador extends Thread{

	/**
	 * Prioridad por defecto del hilo controlador.
	 */
	private final int PRIORIDAD;
	
	/**
	 * Semaforo de un permiso (mutex) para el control concurrente.
	 */
	private Semaphore mutex;
	
	/**
	 * Referencia al objeto que se encarga de elaborar el plan para alcanzar la meta 
	 * impuesta para el robot.
	 */
	private Planificacion planificacion;
	
	/**
	 * Plan elaborado para el robot.
	 */
	private Plan plan;
	
	/**
	 * Lista de predicados que definen el estado actual del modelo del mundo.
	 */
	private LinkedList<Predicado> estado_actual;
	
	/**
	 * Mapa del modelo del mundo que representa las localizaciones y conexiones entre ellas.
	 */
	private Mapa mapa;
	
	/**
	 * Flujo de comunicacion (escritura) con el hilo secuenciador.
	 */
	private PipedWriter com_secuenciador;
	
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public HiloPlanificador(){
		this.PRIORIDAD = 3;
	}
	
	/**
	 * Inicializa el planificador y sus comunicaciones, crea una instancia de planificacion 
	 * y el mapa, asigna al estado actual del modelo del mundo el estado inicial de este e 
	 * inicializa el plan a null, en espera de la planificacion del mismo.
	 * 
	 * @param robot El robot del sistema que utiliza la arquitectura
	 * @param estado_inicial La lista de predicados ciertos en el estado inicial del sistema
	 * @param mapa El nombre del fichero de configuracion del mapa del entorno
	 * @param dominio El nombre del fichero de configuracion del dominio del planificador
	 * @param com_s El flujo de escritura por el que se manda informacion al secuenciador
	 */
	public HiloPlanificador(Robot robot, LinkedList<Predicado> estado_inicial, String mapa, String dominio, PipedReader com_s){
		this.PRIORIDAD = 3;
		this.setPriority(this.PRIORIDAD);
		this.mapa = new Movimientos(mapa);
		this.estado_actual = estado_inicial;
		this.planificacion = new Planificacion(dominio, this.estado_actual);
		this.plan = null;
		/* TODO Descomentar para utilizar ejecucion concurrente */
		/*try {
			this.com_secuenciador = new PipedWriter(com_s);
		} catch (IOException e) {
			System.out.println("ERROR al crear el flujo de comunicacion con el hilo secuenciador");
		}*/
	}
	
	
	/**
	 * Devuelve el plan elaborado por el planificador para llevar a cabo por el robot.
	 * 
	 * @return El plan elaborado
	 */
	public Plan getPlan(){
		return this.plan;
	}
	
	/**
	 * Devuelve el objeto que permite realizar la planificacion del plan.
	 * 
	 * @return El objeto de planificacion
	 */
	public Planificacion getPlanificacion(){
		return this.planificacion;
	}
	
	/**
	 * Devuelve el estado actual del entorno en el modelo del mundo.
	 * 
	 * @return la lista de predicados que son ciertos para el momento actual en el modelo del mundo
	 */
	public LinkedList<Predicado> getEstadoActual(){
		return this.estado_actual;
	}
	
	/**
	 * Devuelve el mapa que representa las localizaciones y conexiones del entorno del robot.
	 * 
	 * @return El mapa del entorno
	 */
	public Mapa getMapa(){
		return this.mapa;
	}
	
	/**
	 * Escribe una serie de caracteres en el buffer del flujo de comunicacion
	 * con el hilo controlador.
	 * 
	 * @param mensaje La cadena de caracteres a escribir
	 * 
	 */
	public void escribirMensaje(String mensaje){
		try{
			this.com_secuenciador.write(mensaje);
		} catch(IOException e){
			System.out.println("ERROR al escribir en el comunicador con el hilo secuenciador.");
		}
	}
	
	/**
	 * Metodo de ejecucion del hilo planificador, heredado de la clase Thread.
	 * 
	 * TODO Rellenar el metodo cuando se implemente la capa superior de la arquitectura
	 */
	public void run(){
		
	}
}
