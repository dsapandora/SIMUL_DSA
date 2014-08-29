package infraestructura;

import java.util.concurrent.Semaphore;

import planificador.gestorPlanes.HiloPlanificador;
import secuenciador.interprete.HiloSecuenciador;
import controlador.gestorHabilidades.HiloControlador;

/**
 * Clase que representa el software de control del robot, es decir, la 
 * arquitectura 3T de la que hace uso. Esta clase encapsula la creaci�n 
 * de los diferentes hilos de ejecuci�n de la arquitectura, lo que 
 * simplifica la conexi�n de �sta con las diversas aplicaciones cliente.
 * 
 * @author Jos� Luis D�az Cebri�n
 * @version 1.0
 * 
 */
public class Arquitectura {

	/**
	 * Hilo de ejecuci�n de la capa inferior de la arquitectura.
	 */
	private HiloControlador controlador;
	
	/**
	 * Hilo de ejecuci�n de la capa intermedia de la arquitectura.
	 */
	private HiloSecuenciador secuenciador;
	
	/**
	 * Hilo de ejecuci�n de la capa superior de la arquitectura.
	 */
	private HiloPlanificador planificador;
	
	/**
	 * Sem�foro de un permiso (mutex) para el control concurrente.
	 */
	private Semaphore mutex;
	private Robot robot;
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Arquitectura(){}
	
	/**
	 * Inicializa los elementos de la arquitectura para que �stos puedan
	 * acceder a las operaciones necesarias sobre el robot y configura el 
	 * modo de ejecuci�n, ya sea sobre simulador o sobre el robot real.
	 * 
	 * @param robot Instancia �nica del robot que hace uso de esta arquitectura
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
	 * Pone en ejecuci�n los tres hilos correspondientes a las tres capas de 
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
	 * M�todo para pruebas.
	 */
	public HiloControlador getControlador(){
		return this.controlador;
	}
	
	/**
	 * M�todo para pruebas.
	 */
	public HiloSecuenciador getSecuenciador(){
		return this.secuenciador;
	}
	
	/**
	 * M�todo para pruebas.
	 */
	public HiloPlanificador getPlanificador(){
		return this.planificador;
	}
	
}
