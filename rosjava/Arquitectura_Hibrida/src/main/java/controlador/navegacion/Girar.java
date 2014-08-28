package controlador.navegacion;

import controlador.gestorHabilidades.Habilidad;
import infraestructura.*;

/**
 * Clase que hereda de Habilidad y define las caracteristicas 
 * propias de esta habilidad e implementa su bucle de ejecucion 
 * y la condicion del evento de finalizacion.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Girar extends Habilidad{

	/**
	 * Indica la direccion del giro que efectuara el robot. A "true" si se desea 
	 * girar a la izquierda.
	 */
	private boolean izquierda;
	
	/**
	 * Indica el factor (de 1 a 5) con el que se desea efectuar el giro. A mayor 
	 * factor, mas velocidad y fuerza en el giro.
	 */
	private int factor;
	
	/**
	 * Indica el angulo que se desea girar (de 0 a 360).
	 */
	private int angulo_final;
	
	/**
	 * Indica el angulo que ya se lleva girado desde que se empezo a ejecutar la 
	 * habilidad.
	 */
	private int angulo_actual;
	
	
	/**
	 * Inicializa los atributos de la habilidad segun resultados obtenidos en experimentaciones
	 * previas de robotica.
	 * 
	 * @param robot El robot del sistema que hace uso de la habilidad
	 * @param sim A "true" si se utiliza la arquitectura sobre simulador
	 * @param izda A "true" si se desea girar a la izquieda, a "false" en caso contrario
	 * @param fact La fuerza (de 1 a 5) con la que se realiza el giro
	 * @param ang El angulo en grados que se desea girar
	 * 
	 */
	public Girar(Robot robot, boolean sim, boolean izda, int fact, int ang){
		super("Girar", robot, sim);
		this.izquierda = izda;
		this.factor = fact;
		this.angulo_final = ang;
		this.angulo_actual = 0;
	}
	
	
	/**
	 * Devuelve la direccion del giro que esta efectuando el robot.
	 * 
	 * @return "true" si el giro es a la izquierda, "false" si es a la derecha
	 * 
	 */
	public boolean getGiro(){
		return this.izquierda;
	}
	
	/**
	 * Devuelve el angulo total del giro que esta efectuando el robot.
	 * 
	 * @return El angulo objetivo del giro
	 * 
	 */
	public int getAngulo(){
		return this.angulo_final;
	}
	
	/**
	 * Asigna una direccion al giro que va a efectuar el robot.
	 * 
	 * @param izda A "true" si se desea un giro a la izquierda, "false" en caso contrario
	 */
	public void setGiro(boolean izda){
		this.izquierda = izda;
	}
	
	/**
	 * Asigna un nuevo angulo al giro que va a efectuar el robot.
	 * 
	 * @param ang El nuevo angulo de giro
	 */
	public void setAngulo(int ang){
		this.angulo_final = ang;
	}
	
	/**
	 * @see Habilidad#ejecutar()
	 */
	public void ejecutar(){
		int[] ruedas = new int[2]; //Izda-Dcha
		
		if(super.getSim())
			ruedas = ejecSimulador();
		else
			ruedas = ejecRobotReal();
		
		Motor m = (Motor) super.getRobot().getActuador("motor");
		m.setRuedas(ruedas[0], ruedas[1]);	
	}
	
	/**
	 * Da un valor a las ruedas del robot teniendo en cuenta los umbrales
	 * y pequenas diferencias en el algoritmo para el simulador.
	 * 
	 * @return Las velocidades a asignar a las ruedas del robot
	 */
	public int[] ejecSimulador(){
		
		int ruedas[] = new int[2]; //Izda-Dcha
		
		/* Por defecto, giro a la izquierda */
		switch(factor){
		case 1: //Giro muy debil
			ruedas[0] = 0;
			ruedas[1] = 10;
			break;
		case 2: //Giro debil
			ruedas[0] = 0;
			ruedas[1] = 20;
			break;
		case 3: //Giro normal
			ruedas[0] = 0;
			ruedas[1] = 30;
			break;
		case 4: //Giro fuerte
			ruedas[0] = 0;
			ruedas[1] = 40;
			break;
		case 5: //Giro muy fuerte
			ruedas[0] = 0;
			ruedas[1] = 50;
			break;
		default:
			ruedas[0] = 0;
			ruedas[1] = 0;
		}
		
		if(!izquierda){ //Si giro a la derecha, cambio de ruedas
			int aux = ruedas[0];
			ruedas[0] = ruedas[1];
			ruedas[1] = aux;
		}
		
		return ruedas;
	}
	
	/**
	 * Da un valor a las ruedas del robot teniendo en cuenta los umbrales
	 * y pequenas diferencias en el algoritmo para el robot real sobre ARIA.
	 * 
	 * @return Las velocidades a asignar a las ruedas del robot
	 */
	public int[] ejecRobotReal(){
		
		int ruedas[] = new int[2]; //Izda-Dcha
		
		/* Por defecto, giro a la izquierda */
		switch(factor){
		case 1: //Giro muy debil
			ruedas[0] = 0;
			ruedas[1] = 20;
			break;
		case 2: //Giro debil
			ruedas[0] = 0;
			ruedas[1] = 40;
			break;
		case 3: //Giro normal
			ruedas[0] = 0;
			ruedas[1] = 60;
			break;
		case 4: //Giro fuerte
			ruedas[0] = 0;
			ruedas[1] = 80;
			break;
		case 5: //Giro muy fuerte
			ruedas[0] = 0;
			ruedas[1] = 100;
			break;
		default:
			ruedas[0] = 0;
			ruedas[1] = 0;
		}
		
		if(!izquierda){ //Si giro a la derecha, cambio de ruedas
			int aux = ruedas[0];
			ruedas[0] = ruedas[1];
			ruedas[1] = aux;
		}
		
		return ruedas;
	}
	
	/**
	 * @see Habilidad#comprobarExito()
	 */
	public boolean comprobarExito(){
		boolean exito = false;
		
		if(super.getSim())
			exito = exitoSimulador();
		else
			exito = exitoRobotReal();
		
		return exito;
	}

	/**
	 * Comprueba la condicion de exito de la habilidad teniendo en cuenta los 
	 * umbrales y pequenas diferencias en el algoritmo para el simulador.
	 * 
	 * @return "true" si se ha ejecutado con exito la habilidad, "false" en caso contrario
	 */
	public boolean exitoSimulador(){
		boolean exito = false;
	
		if(this.angulo_actual>=this.angulo_final){
			exito = true;
			this.angulo_actual = 0;
		} else{
			/* Sumar al angulo a razon de factor*6 */
			this.angulo_actual = this.angulo_actual + (this.factor * 6);
		}
		
		return exito;
	}
	
	/**
	 * Comprueba la condicion de exito de la habilidad teniendo en cuenta los 
	 * umbrales y pequenas diferencias en el algoritmo para el robot real sobre ARIA.
	 * 
	 * @return "true" si se ha ejecutado con exito la habilidad, "false" en caso contrario
	 */
	public boolean exitoRobotReal(){
		boolean exito = false;
		
		if(this.angulo_actual>=this.angulo_final){
			exito = true;
			this.angulo_actual = 0;
		} else{
			/* Sumar el angulo a razon del factor */
			this.angulo_actual = this.angulo_actual + this.factor;
		}
		
		return exito;
	}
	
}
