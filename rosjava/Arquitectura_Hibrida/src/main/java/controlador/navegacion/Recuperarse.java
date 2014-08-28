package controlador.navegacion;

import utilidades.Funciones;
import infraestructura.Bumpers;
import infraestructura.Motor;
import infraestructura.Robot;
import infraestructura.Sonar;
import controlador.gestorHabilidades.Habilidad;

/**
 * Clase que hereda de Habilidad y define las caracteristicas 
 * propias de esta habilidad e implementa su bucle de ejecucion 
 * y la condicion del evento de finalizacion.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Recuperarse extends Habilidad{
	
	/**
	 * Umbral maximo de acercamiento del robot a un obstaculo. Si el robot se encuentra a menor distancia, 
	 * es necesario seguir retrocediendo.
	 */
	private double UMBRAL;
	
	
	/**
	 * Inicializa los atributos de la habilidad segun resultados obtenidos en experimentaciones
	 * previas de robotica.
	 * 
	 * @param robot El robot del sistema que hace uso de la habilidad
	 * @param sim A "true" si se utiliza la arquitectura sobre simulador
	 * 
	 */
	public Recuperarse(Robot robot, boolean sim){
		super("Recuperarse", robot, sim);
		if(sim)
			this.UMBRAL = 100.0;
		else
			this.UMBRAL = 400.0;
	}
	
	
	/**
	 * Devuelve el valor minimo al que el robot debe retroceder.
	 * 
	 * @return La distancia minima con el obstaculo para poder girar
	 * 
	 */
	public double getUmbral(){
		return this.UMBRAL;
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
		
		int[] ruedas = new int[2]; //Izda-Dcha
		Bumpers b = (Bumpers) super.getRobot().getSensor("bumpers");
		int choque = b.getValor();
		
		switch(choque){
		case 100: //Giro extremo a la derecha
			ruedas[0] = -20;
			ruedas[1] = -80;
			break;
		case 200: //Giro a la izquierda
			ruedas[0] = -50;
			ruedas[1] = -10;
			break;
		case 300: //Giro a la derecha
			ruedas[0] = -10;
			ruedas[1] = -50;
			break;
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
		
		int[] ruedas = new int[2]; //Izda-Dcha
		Bumpers b = (Bumpers) super.getRobot().getSensor("bumpers");
		int choque = b.getValor();
		
		switch(choque){
		case 0: //Sin choque, no hago nada
			break;
		case 512: //Giro suave a la derecha
			ruedas[0] = -30;
			ruedas[1] = -80;
			break;
		case 1024: //Giro a la derecha
			ruedas[0] = -50;
			ruedas[1] = -100;
			break;
		case 2048: //Giro extremo a la derecha
			ruedas[0] = -100;
			ruedas[1] = -200;
			break;
		case 4096: //Giro a la izquierda
			ruedas[0] = -100;
			ruedas[1] = -50;
			break;
		case 8192: //Giro suave a la izquierda
			ruedas[0] = -80;
			ruedas[1] = -30;
			break;
		default: //Si choca con mas de 1 bumper, giro suave
			ruedas[0] = -200;
			ruedas[1] = -200;
			break;
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
		
		Sonar s = (Sonar) super.getRobot().getSensor("sonar");
		double[] todos = new double[s.getNumSonar()];
		for(int i=0; i<todos.length; i++)
			todos[i] = s.getValorSonar(i);
		/*double[] delanteros = {todos[0], todos[1], todos[2], todos[13], todos[14], todos[15]}; CORRECCION*/
                double[] delanteros = {todos[0], todos[1], todos[2], todos[3], todos[4], todos[5],todos[6]};
		double max = Funciones.maximo(delanteros);
		
		if(max<this.UMBRAL)
			exito = true;
		
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
		
		Sonar s = (Sonar) super.getRobot().getSensor("sonar");
		double[] valores = new double[s.getNumSonar()];
		for(int i=1; i<valores.length-1; i++)
			valores[i] = s.getValorSonar(i);
		double[] delanteros = {valores[1], valores[2], valores[3], valores[4], valores[5], valores[6]};
		double min = Funciones.minimo(delanteros);
		
		if(min>this.UMBRAL)
			exito = true;
		
		return exito;
	}
	

}
