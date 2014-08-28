package infraestructura;

import java.util.concurrent.Semaphore;

import planificador.gestorPlanes.HiloPlanificador;
import secuenciador.interprete.HiloSecuenciador;
import controlador.gestorHabilidades.HiloControlador;

/**
 * Clase que representa el software de control del robot, es decir, la 
 * arquitectura 3T de la que hace uso. Esta clase encapsula la creación 
 * de los diferentes hilos de ejecución de la arquitectura, lo que 
 * simplifica la conexión de ésta con las diversas aplicaciones cliente.
 * 
 * @author José Luis Díaz Cebrián
 * @version 1.0
 * 
 */
public class Arquitectura {

	/**
	 * Hilo de ejecución de la capa inferior de la arquitectura.
	 */
	private HiloControlador controlador;
	
	/**
	 * Hilo de ejecución de la capa intermedia de la arquitectura.
	 */
	private HiloSecuenciador secuenciador;
	
	/**
	 * Hilo de ejecución de la capa superior de la arquitectura.
	 */
	private HiloPlanificador planificador;
	
	/**
	 * Semáforo de un permiso (mutex) para el control concurrente.
	 */
	private Semaphore mutex;
	private Robot robot;
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Arquitectura(){}
	
	/**
	 * Inicializa los elementos de la arquitectura para que éstos puedan
	 * acceder a las operaciones necesarias sobre el robot y configura el 
	 * modo de ejecución, ya sea sobre simulador o sobre el robot real.
	 * 
	 * @param robot Instancia única del robot que hace uso de esta arquitectura
	 * @param sim A "true" si el robot se va a simular, a "false" si el robot es real
	 * 
	 */
	public Arquitectura(Robot robot, boolean sim){
                 this.robot=robot;
		this.mutex = new Semaphore(1);
		/* TODO Descomentar cuando se implemente la capa superior de la arquitectura 
		 * e indicar los argumentos correctos que utilice el hilo */
		//this.planificador = new HiloPlanificador(robot, null, fichero, null);
		this.secuenciador = new HiloSecuenciador(robot, null, null);
		this.controlador = new HiloControlador(robot, this.secuenciador, this.mutex, sim, null);
		this.ejecutar();
	}
	
	
	/**
	 * Pone en ejecución los tres hilos correspondientes a las tres capas de 
	 * la arquitectura.
	 */
	public void ejecutar(){
		/* TODO Descomentar cuando se implemente la capa superior de la arquitectura */
		System.out.println("INICIA EL EXPERIMENTO");
             //   this.planificador.start();
		//this.secuenciador.start();
                robot.getMotor().publicador();
		this.controlador.start();
	}
	
	
	
	
	/**
	 * Método para pruebas.
	 */
	public HiloControlador getControlador(){
		return this.controlador;
	}
	
	/**
	 * Método para pruebas.
	 */
	public HiloSecuenciador getSecuenciador(){
		return this.secuenciador;
	}
	
	/**
	 * Método para pruebas.
	 */
	public HiloPlanificador getPlanificador(){
		return this.planificador;
	}
	
}
