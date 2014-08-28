package controlador.gestorHabilidades;

import infraestructura.Robot;

import java.io.IOException;
import java.io.PipedReader;
import java.io.PipedWriter;
import java.util.LinkedList;
import java.util.concurrent.Semaphore;

import planificador.gestorPlanes.Meta;

import secuenciador.interprete.HiloSecuenciador;
import secuenciador.interprete.Tarea;

/** 
 * Thread que mantiene la ejecucion paralela del subsistema 
 * Controlador, ejecutando la parte reactiva de la arquitectura 
 * (habilidades). Gestiona las comunicaciones con el resto de 
 * hilos de ejecucion. 
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class HiloControlador extends Thread{

	/**
	 * Prioridad por defecto del hilo controlador.
	 */
	private final int PRIORIDAD;
	
	/**
	 * Semaforo de un permiso (mutex) para el control concurrente.
	 */
	private Semaphore mutex;
	
	/**
	 * Referencia al secuenciador del sistema.
	 */
	private HiloSecuenciador secuenciador;
	
	/**
	 * Conjunto de todas las habilidades que puede llevar a cabo el robot.
	 */
	private Habilidades todas;
	
	/**
	 * Conjunto de las habilidades activadas en el momento actual por el secuenciador.
	 */
	private Habilidades conjunto_en_uso;
	
	/**
	 * Habilidad que esta ejecutando el robot en el momento actual.
	 */
	private Habilidad en_ejecucion;
	
	/**
	 * Tarea que esta llevando a cabo el robot en el momento actual.
	 */
	private Tarea tarea_actual;
	
	/**
	 * Indica si se ha alcanzado la submeta de la tarea actual.
	 */
	private boolean exito;
	
	/**
	 * Flujo de comunicacion (lectura) con el hilo secuenciador.
	 */
	private PipedReader com_secuenciador;
	private Robot robot;
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public HiloControlador(){
		this.PRIORIDAD = 5;
	}
	
	/**
	 * Inicializa el controlador y sus comunicaciones, crea una instancia de cada habilidad 
	 * del sistema y asigna una tarea inicial especial de espera hasta que el robot reciba 
	 * una tarea real a llevar a cabo.
	 * 
	 * @param robot El robot del sistema que utiliza la arquitectura
	 * @param sem El mutex compartido por todos los hilos de la arquitectura
	 * @param sim A "true" si se utiliza la arquitectura sobre simulador
	 * @param com El flujo de escritura por el que manda informacion el secuenciador
	 * 
	 */
	public HiloControlador(Robot robot, HiloSecuenciador sec, Semaphore sem, boolean sim, PipedWriter com){
		this.PRIORIDAD = 5;
                this.robot=robot;
		this.setPriority(this.PRIORIDAD);
		this.mutex = sem;
		this.secuenciador = sec;
		this.todas = new Habilidades(true, robot, sim);
		this.conjunto_en_uso = new Habilidades(false, robot, sim);
		this.en_ejecucion = null;
		this.tarea_actual = new Tarea("Inicio", null);
		this.exito = false;
		/* TODO Descomentar para utilizar ejecucion concurrente */
		/*try {
			this.com_secuenciador = new PipedReader(com, 1024);
		} catch (IOException e) {
			System.out.println("ERROR al crear el flujo de comunicacion con el hilo secuenciador");
		}*/
	}
	

	/**
	 * Devuelve el conjunto de todas las habilidades que puede desarrollar el robot.
	 * 
	 * @return El conjunto de habilidades
	 * 
	 */
	public Habilidades getHabilidades(){
		return this.todas;
	}
	
	/**
	 * Devuelve la habilidad que esta ejecutando el robot en el momento actual.
	 * 
	 * @return La habilidad en ejecucion
	 * 
	 */
	public Habilidad getHabilidadActual(){
		return this.en_ejecucion;
	}
	
	/**
	 * Devuelve la tarea actual que esta llevando a cabo el robot.
	 * 
	 * @return La tarea actual
	 * 
	 */
	public Tarea getTareaActual(){
		return this.tarea_actual;
	}
	
	/**
	 * Devuelve el estado en el que se encuentra la tarea en el momento actual.
	 * 
	 * @return "true" si el robot ha alcanzado la submeta de la tarea actual encomendada
	 *
	 */
	public boolean getExito(){
		return this.exito;
	}
	
	/**
	 * Actualiza el estado en el que se encuentra la tarea en el momento actual, comprobando
	 * si se ha alcanzado la submeta de dicha tarea.
	 */
	public void setExito(boolean exito){
		this.exito = exito;
	}
	
	/**
	 * Asigna una nueva tarea a llevar a cabo por el robot, inicializando su estado en consecuencia.
	 * 
	 * @param tarea La nueva tarea actual
	 * 
	 */
	public void cambiarTarea(Tarea tarea){
		this.tarea_actual = tarea;
		this.exito = false;
	}
	
	/**
	 * Asigna una nueva habilidad para poner en ejecucion. Se usa conjuntamente con el 
	 * metodo setTareaActual() cuando se cambia de tarea.
	 * 
	 * @param id El identificador (nombre) de la habilidad
	 * 
	 */
	public void setHabilidadActual(String id){
		this.en_ejecucion = this.conjunto_en_uso.getHabilidad(id);
	}
	
	/**
	 * Anade una nueva habilidad al conjunto de las habilidades activadas por el secuenciador.
	 * 
	 * @param id El identificador (nombre) de la habilidad
	 * 
	 */
	public void anyadirHabilidad(String id){
                System.out.println("AGREGA LAS HABILIDADES");
		Habilidad hab = this.todas.getHabilidad(id);
		this.conjunto_en_uso.anyadirHabilidad(hab);
	}
	
	/**
	 * Pone en ejecucion la habilidad concreta dentro del conjunto activo que debe ser ejectuada
	 * a continuacion.
	 */
	public void ejecutarHabilidad(){
                System.out.println(this.en_ejecucion.getId()+" "+robot.getBumpers().getValor());
                robot.getMotor().publicador();
		this.en_ejecucion.ejecutar();
	}

	/**
	 * Actualiza el conjunto activo de habilidades con nuevas habilidades indicadas por el
	 * secuenciador.
	 * 
	 * @param conjunto La lista de los nombres de las nuevas habilidades activas
	 * 
	 */
	public void activarConjunto(LinkedList<String> conjunto){
		this.conjunto_en_uso.eliminarTodas();
		for(int i=0; i<conjunto.size(); i++){
                        System.out.println("IMPRIME "+i+" "+conjunto.get(i));
                        //if(conjunto.get(i)!="IrA")
			  this.conjunto_en_uso.anyadirHabilidad(this.todas.getHabilidad(conjunto.get(i)));
                        
		}
	}
	
	/**
	 * Metodo que realiza el conjunto de acciones que permite dar un nuevo paso en la 
	 * secuencia de tareas, es decir, cuando una tarea se lleva a cabo con exito, toma la 
	 * siguiente de la agenda, se lo comunica al hilo controlador y se selecciona una nueva 
	 * habilidad en funcion del RAP asignado a la nueva tarea.
	 * 
	 * @param pos Posicion de la nueva tarea dentro de la secuencia de tareas
	 */
	public void nuevoPasoSecuencia(int pos){
		this.cambiarTarea(this.secuenciador.getAgenda().siguiente());
		LinkedList<String> habs = this.secuenciador.getSecuenciacion().getSolucion().get(pos).getHabilidades();
		this.activarConjunto(habs);
		String hab = this.secuenciador.getSecuenciacion().seleccionarHabilidad(
				this.secuenciador.getSecuenciacion().getSolucion().get(pos), 
				this.secuenciador.getMonitor().getEstado(), 
				this.secuenciador.getMonitor().getPosicion());
		this.setHabilidadActual(hab);
                robot.getMotor().publicador();
	}
	
	/**
	 * Metodo que realiza las operaciones necesarias para cambiar la secuencia actual de 
	 * tareas tras detectar una colision durante la ejecucion de la tarea actual.
	 */
	public void informarChoque(){
		this.secuenciador.getAgenda().anyadirTarea(this.tarea_actual, 0);
		this.secuenciador.getAgenda().anyadirTarea(new Tarea("Retroceder", new Meta("SuperarChoque", null)), 0);
		this.recuperarSecuencia();
	}
	
	/**
	 * Metodo que se encarga de modificar la secuencia de tareas tras ser informado de 
	 * un fallo en la ejecucion de la tarea actual. Coloca en el primer lugar de la agenda 
	 * una tarea de recuperacion y actualiza el conjunto de habilidades y la habilidad actual
	 * para llevar a cabo la recuperacion del robot.
	 */
	public void recuperarSecuencia(){
		this.cambiarTarea(this.secuenciador.getAgenda().siguiente());
		LinkedList<String> habs = new LinkedList<String>();
		habs.add("Recuperarse");
		this.activarConjunto(habs);
		this.setHabilidadActual("Recuperarse");
	}

	/**
	 * Lee todos los caracteres que se encuentren en el buffer del flujo de comunicacion
	 * con el hilo secuenciador.
	 * 
	 * @return La cadena de caracteres leidos
	 * 
	 */
	public String leerMensaje(){
		String mensaje = "";
		char c;
		try{
			while(this.com_secuenciador.ready()){
				c = (char) this.com_secuenciador.read();
				mensaje = mensaje + c;
			}
		} catch(IOException e){
			System.out.println("ERROR al leer del comunicador con el hilo secuenciador.");
		}
		return mensaje;
	}

	/**
	 * Metodo de ejecucion del hilo controlador, heredado de la clase Thread.
	 * 
	 * TODO Este metodo sirve para la ejecucion secuencial. Para usar la ejecucion concurrente, comentar este metodo y descomentar el siguiente
	 */
	public void run(){
	
		this.en_ejecucion = this.todas.getHabilidad("Esperar");

		/* Bucle hasta que se tiene un plan */
		while(this.secuenciador.getPlan()==null){
			System.out.println("Esperando planificacion...");
		}

	
		/* Se crea la secuencia de tareas */
		this.secuenciador.getSecuenciacion().actualizarPlan(this.secuenciador.getPlan());
		this.secuenciador.getSecuenciacion().secuenciar();
		this.secuenciador.getAgenda().crearAgenda(
				this.secuenciador.getPlan(), this.secuenciador.getSecuenciacion().getSecuencia());
		
		/* Se actualiza la informacion del robot mediante el monitor */
		this.secuenciador.getMonitor().actualizar();
		
		/* Se inicializa el controlador con la primera tarea y habilidad */
		int posicion_tarea = 0;
		this.nuevoPasoSecuencia(posicion_tarea);
		
		try {
			sleep(500);
		} catch (InterruptedException e) {
			System.out.println("ERROR del controlador mientras dormia");
		}
		
		/* Bucle hasta que todas las tareas finalizan */
		while(this.tarea_actual.getNombre().compareTo("Fin")!=0){
			
			/* Se comprueba si la tarea actual ha finalizado */
                                        robot.getMotor().publicador();
                                        if(robot.getBumpers().getValor()!=0)
                                                 informarChoque();
			this.setExito(this.en_ejecucion.comprobarExito());
			if(this.exito){
				/* Se toma una nueva tarea, si es que quedan pendientes */
				if(this.secuenciador.getAgenda().hayMasTareas()){
					posicion_tarea++;
					this.nuevoPasoSecuencia(posicion_tarea);
				} else{
					/* El plan se ha llevado a cabo con exito, por lo que se registra la solucion */
					this.secuenciador.getSecuenciacion().getHistorial().guardarPlan(
							this.secuenciador.getPlan(), 
							this.secuenciador.getSecuenciacion().getSolucion(), 
							this.secuenciador.getSecuenciacion().getSecuencia());
					this.cambiarTarea(new Tarea("Fin", null));
				}
			} else{
				/* Se ejecuta un nuevo paso de la habilidad */
				this.ejecutarHabilidad();
			}
			
			/* Se actualiza la informacion del robot mediante el monitor */
			this.secuenciador.getMonitor().actualizar();
			
			try {
				sleep(300);
			} catch (InterruptedException e) {
				System.out.println("ERROR del controlador mientras dormia");
			}
		}

		/* Se indica al robot su finalizacion mediante la habilidad Finalizar */
		this.en_ejecucion = this.todas.getHabilidad("Finalizar");
		this.ejecutarHabilidad();
		
	}
	
	/**
	 * Metodo de ejecucion del hilo controlador, heredado de la clase Thread.
	 * 
	 * TODO Este metodo sirve para la ejecucion concurrente. Para usar la ejecucion concurrente, comentar este metodo y descomentar el siguiente
	 */
	/*public void run(){
	
		this.en_ejecucion = this.todas.getHabilidad("Esperar");

		// Bucle hasta que se tiene un plan
		while(this.secuenciador.getPlan()==null){
			System.out.println("Esperando planificacion...");
		}
	
		try {
			sleep(500);
		} catch (InterruptedException e) {
			System.out.println("ERROR del controlador mientras dormia");
		}
		
		// Bucle hasta que todas las tareas finalizan
		while(this.tarea_actual.getNombre().compareTo("Fin")!=0){
			
			// Se comprueba si la tarea actual ha finalizado
			boolean ex = this.en_ejecucion.comprobarExito();
			try {
				this.mutex.acquire();
				this.setExito(ex);
				if(!this.exito){
					this.ejecutarHabilidad();
				}
			} catch (InterruptedException e1) {
				System.out.println("ERROR del mutex al hacer lock");
			}
			this.mutex.release();
			
			try {
				sleep(500);
			} catch (InterruptedException e) {
				System.out.println("ERROR del controlador mientras dormia");
			}
			
		}

		// Se indica al robot su finalizacion mediante la habilidad Finalizar
		this.en_ejecucion = this.todas.getHabilidad("Finalizar");
		this.ejecutarHabilidad();
		
	}*/
	
	/**
	 * Metodo para pruebas.
	 * 
	 * TODO Descomentar la prueba que se desee ejecutar
	 */
	public void run_pruebas(){
		
		/* Operaciones del controlador para la prueba 1 */
		/*anyadirHabilidad("SeguirPared");
		this.en_ejecucion = this.todas.getHabilidad("SeguirPared");*/
		
		/* Operaciones del controlador para la prueba 2 */
		/*anyadirHabilidad("Avanzar");
		anyadirHabilidad("Recuperarse");
		this.en_ejecucion = this.todas.getHabilidad("Avanzar");*/
		
		/* Operaciones del controlador para la prueba 3 */
		/*anyadirHabilidad("Avanzar");
		anyadirHabilidad("EvitarObstaculo");
		this.en_ejecucion = this.todas.getHabilidad("Avanzar");*/
	
	}
	
	
}
