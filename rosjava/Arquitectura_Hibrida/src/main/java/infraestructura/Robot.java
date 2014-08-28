package infraestructura;

import org.ros.node.ConnectedNode;

/**
 * Clase que representa el robot fsico, con todas sus caractersticas,
 * y proporciona el acceso a stas y al estado del propio robot. Sirve
 * como enlace nico entre la arquitectura del sistema y el cliente
 * software.
 * 
 * @author Jos Luis Daz Cebrin
 * @version 1.0
 * 
 */
public class Robot{

	/**
	 * Estado actual del robot dentro del modelo del mundo.
	 */
	private Estado estado;
	
	/**
	 * Sensor de tipo bumpers.
	 */
	private Bumpers bumpers;
	
	/**
	 * Sensor de tipo snar.
	 */
	private Sonar sonar;
	
		/**
	 * Sensor de tipo camara.
	 */
	private Camara camara;
	/**
	 * Sensor de tipo lser.
	 */
	private Laser laser;
	
	/**
	 * Actuador de tipo motor.
	 */
	private Motor motor;
	private Motor motor2;
	
	/**
	 * Elementos de la arquitectura 3T, de la que hace uso el robot.
	 */
	private Arquitectura arquitectura;
	private ConnectedNode node;
	
	/**
	 * Inicializa por defecto con los atributos a null.
	 */
	public Robot(){}
	
	/**
	 * Inicializa el robot con un estado inicial y habilita los sensores y actuadores
	 * que va a utilizar durante la ejecucin.
	 * 
	 * @param estado El estado inicial del robot
	 * @param sensores La lista de sensores que se van a habilitar
	 * @param actuadores La lista de actuadores que se van a habilitar
	 * @param sim A "true" si el robot se va a simular, a "false" si el robot es real
	 * 
	 */
	public Robot(Estado estado, Sensor[] sensores, Actuador[] actuadores,final ConnectedNode node ,boolean sim){
		this.node=node;
		inicializar(estado, sensores, actuadores);
		this.arquitectura = new Arquitectura(this, sim);
	}
	
	
	/**
	 * Devuelve el estado actual del robot dentro del modelo del mundo.
	 * 
	 * @return El estado actual del robot
	 * 
	 */
	public Estado getEstado(){
		return this.estado;
	}
	
	/**
	 * Devuelve la referencia al sensor de tipo bumpers del robot.
	 * 
	 * @return El sensor bumpers
	 * 
	 */
	public Bumpers getBumpers(){
		return this.bumpers;
	}
	
	/**
	 * Devuelve la referencia al sensor de tipo snar del robot.
	 * 
	 * @return El sensor snar
	 * 
	 */
	public Sonar getSonar(){
		return this.sonar;
	}
	
	/**
	 * Devuelve la referencia al sensor de tipo lser del robot.
	 * 
	 * @return El sensor lser
	 * 
	 */
	public Laser getLaser(){
		return this.laser;
	}
	
	/**
	 * Devuelve la referencia al actuador de tipo motor del robot.
	 * 
	 * @return El actuador motor
	 * 
	 */
	public Motor getMotor(){
		return this.motor;
	}
	
	/**
	 * Devuelve la referencia a un sensor conreto del robot.
	 * 
	 * @param id El identificador (nombre) del sensor
	 * @return El sensor
	 * 
	 */
	public Sensor getSensor(String id){
		Sensor sensor = null;
		if(id.compareTo(this.sonar.getId())==0)
			sensor = this.sonar;
		else if(id.compareTo(this.laser.getId())==0)
			sensor = this.laser;
		else if(id.compareTo(this.camara.getId())==0)
			sensor = this.camara;	
		else if(id.compareTo(this.bumpers.getId())==0)
			sensor = this.bumpers;
		else
			System.out.println("ERROR: El identificador " + id + " no se corresponde con ningn sensor.");
		return sensor;
	}
	
	/**
	 * Devuelve la referencia a un actuador conreto del robot.
	 * 
	 * @param id El identificador (nombre) del actuador
	 * @return El actuador
	 * 
	 */
	public Actuador getActuador(String id){
		Actuador actuador = null;
		if(id.compareTo(this.motor.getId())==0)
			actuador = this.motor;
		else
			System.out.println("ERROR: El identificador " + id + " no se corresponde con ningn actuador.");
		return actuador;
	}
	
	/**
	 * Actualiza el estado del robot dentro del modelo del mundo.
	 * 
	 * @param estado El nuevo estado a asignar al robot
	 * 
	 */
	public void setEstado(Estado estado){
		this.estado = estado;
	}
	
	/**
	 * Actualiza la informacin de un sensor concreto del robot.
	 * 
	 * @param id El identificador (nombre) del sensor
	 * @param indice El ndice del snar (-1 si el sensor que se actualiza no es un snar)
	 * @param valor El valor a asignar al sensor
	 */
	public void setSensor(String id, int indice, int valor){
		if(id.compareTo("sonar")==0)
			this.sonar.setValorSonar(indice, valor);
		else if(id.compareTo("laser")==0)
			this.laser.setInfo(valor);
		else if(id.compareTo("camara")==0)
			this.laser.setInfo(valor);	
		else if(id.compareTo("bumpers")==0)
			this.bumpers.setValor(valor);
		else
			System.out.println("ERROR: Tipo de sensor " + id + " desconocido.");
	}
	
	/**
	 * Actualiza la informacin de un actuador concreto del robot.
	 * 
	 * @param id El identificador (nombre) del actuador
	 * @param valores Los valores a asignar al actuador
	 */
	public void setActuador(String id, int[] valores){
		if(id.compareTo("motor")==0)
			this.motor.setRuedas(valores[0], valores[1]);
		else
			System.out.println("ERROR: Tipo de actuador " + id + " desconocido.");
	}
	
	/**
	 * Inicializa el robot con un estado inicial y habilita los sensores y actuadores
	 * que va a utilizar durante la ejecucin.
	 * 
	 * @param estado El estado inicial del robot
	 * @param sensores La lista de sensores que se van a habilitar
	 * @param actuadores La lista de actuadores que se van a habilitar
	 */
	public void inicializar(Estado estado, Sensor[] sensores, Actuador[] actuadores){
		this.estado = estado;
		for(int i=0; i<sensores.length; i++){
			Sensor sensor = sensores[i];
			if(sensor.getId().compareTo("sonar")==0){
				Sonar s = (Sonar) sensor;
				this.sonar = new Sonar(s.getNumSonar(),node);
			}else if(sensor.getId().compareTo("laser")==0){
				this.laser = new Laser(node);
			}else if(sensor.getId().compareTo("bumpers")==0){
				Bumpers b = (Bumpers) sensor;
				this.bumpers = new Bumpers(b.getNumFrontales(), b.getNumTraseros(),node);
		    }else if(sensor.getId().compareTo("camara")==0){
				Camara b = (Camara) sensor;
				this.camara = new Camara(1,node);
			}else
				System.out.println("ERROR: Tipo de sensor " + sensor.getId() + " desconocido. No se ha inicializado el robot.");
		}
		for(int i=0; i<actuadores.length; i++){
			Actuador actuador = actuadores[i];
			if(actuador.getId().compareTo("motor")==0)
				{//this.motor = new Motor(null);
				this.motor = new Motor(this.node);
				this.motor2 = new Motor(this.node);
				}
			else
				System.out.println("ERROR: Tipo de actuador " + actuador.getId() + " desconocido. No se ha inicializado el robot.");
		}
	}
	
	
	
	
	
	/**
	 * Mtodo para pruebas.
	 */
	public Arquitectura getArquitectura(){
		return this.arquitectura;
	}
	
}
