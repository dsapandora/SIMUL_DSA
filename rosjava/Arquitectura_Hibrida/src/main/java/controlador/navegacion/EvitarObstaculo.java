package controlador.navegacion;

import utilidades.Funciones;
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
public class EvitarObstaculo extends Habilidad{

	/**
	 * Indica la direccion por la que se va a evitar el obstaculo. Esta a "true" si se evita por
	 * la izquierda, y a "false" en caso contrario.
	 */
	private boolean izquierda;
	
	/**
	 * Umbral maximo de acercamiento del robot a un obstaculo. Si el robot se encuentra a menor distancia, 
	 * es necesario evitar el obstaculo.
	 */
	private final double UMBRAL;
	
	
	/**
	 * Inicializa los atributos de la habilidad segun resultados obtenidos en experimentaciones
	 * previas de robotica.
	 * 
	 * @param izda A "true" si se desea que el robot esquive obstaculos por la izquierda
	 * @param robot El robot del sistema que hace uso de la habilidad
	 * @param sim A "true" si se utiliza la arquitectura sobre simulador
	 * 
	 */
	public EvitarObstaculo(boolean izda, Robot robot, boolean sim){
		super("EvitarObstaculo", robot, sim);
		this.izquierda = izda;
		if(sim)
			this.UMBRAL = 100.0;
		else
			this.UMBRAL = 300.0;
	}
	
	/**
	 * Devuelve la direccion por la cual el robot evita el obstaculo.
	 * 
	 * @return "true" si esquiva por la izquierda, "false" en caso contrario
	 */
	public boolean getIzda(){
		return this.izquierda;
	}
	
	/**
	 * Devuelve el valor maximo de acercamiento del robot al obstaculo.
	 * 
	 * @return La distancia minima con el obstaculo
	 * 
	 */
	public double getUmbral(){
		return this.UMBRAL;
	}
	
	/**
	 * Asigna una nueva direccion por la que evitar los obstaculos.
	 * 
	 * @param izda A "true" si se va a esquivar los obstaculos por la izquierda, a "false" en caso
	 * contrario
	 * 
	 */
	public void setIzda(boolean izda){
		this.izquierda = izda;
	}
	
	/**
	 * @see Habilidad#ejecutar()
	 */
	public void ejecutar(){
			
		int ruedas[] = new int[2]; //Izda-Dcha
  
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
		Sonar s = (Sonar) super.getRobot().getSensor("sonar");
		double ds_iz = (s.getValorSonar(2)+s.getValorSonar(3))/2;
		double ds_de = (s.getValorSonar(4)+s.getValorSonar(5))/2;
		double ds_fr_izda = Math.max(s.getValorSonar(0), s.getValorSonar(1));
		double ds_fr_dcha = Math.max(s.getValorSonar(6), s.getValorSonar(7));
		double ds_fr = Math.max(ds_fr_izda, ds_fr_dcha);
		
		if(ds_fr_izda>ds_fr_dcha) //Esquivar por la derecha
			this.setIzda(false);
		else //Esquivar por la izquierda
			this.setIzda(true);
  
		if(this.izquierda){
			if (ds_fr > UMBRAL) { //Si tengo el objeto delante, giro a la izquierda
				ruedas[0] = -10;
				ruedas[1] = 50;
			} else if(ds_de > UMBRAL){ //Si no, si estoy cerca del objeto, avanzo
				ruedas[0] = 30;
				ruedas[1] = 30;
			} else{ //Si no, giro a la derecha
				ruedas[0] = 50/2;
				ruedas[1] = 10;
			}
		} else{
			if (ds_fr > UMBRAL) { //Si tengo el objeto delante, giro a la derecha
				ruedas[0] = 50;
				ruedas[1] = -10;
			} else if(ds_iz > UMBRAL){ //Si no, si estoy cerca del objeto, avanzo
				ruedas[0] = 30;
				ruedas[1] = 30;
			} else{ //Si no, giro a la izquierda
				ruedas[0] = 10;
				ruedas[1] = 50/2;
			}
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
		Sonar s = (Sonar) super.getRobot().getSensor("sonar");
		double ds_iz = (s.getValorSonar(0)+s.getValorSonar(1))/2;
		double ds_de = (s.getValorSonar(6)+s.getValorSonar(7))/2;
		double ds_fr_izda = Math.min(s.getValorSonar(2), s.getValorSonar(3));
		double ds_fr_dcha = Math.min(s.getValorSonar(4), s.getValorSonar(5));
		double ds_fr = Math.min(ds_fr_izda, ds_fr_dcha);
  
		if(ds_fr_izda<ds_fr_dcha) //Esquivar por la derecha
			this.setIzda(false);
		else //Esquivar por la izquierda
			this.setIzda(true);
		
		if(this.izquierda){
			if (ds_fr < UMBRAL) { //Si tengo el objeto delante, giro a la izquierda
				ruedas[0] = -20;
				ruedas[1] = 140;
			} else if(ds_de < UMBRAL){ //Si no, si estoy cerca del objeto, avanzo
				ruedas[0] = 60;
				ruedas[1] = 60;
			} else{ //Si no, giro a la derecha
				ruedas[0] = 140/2;
				ruedas[1] = 20;
			}
		} else{
			if (ds_fr < UMBRAL) { //Si tengo el objeto delante, giro a la derecha
				ruedas[0] = 140;
				ruedas[1] = -20;
			} else if(ds_iz < UMBRAL){ //Si no, si estoy cerca del objeto, avanzo
				ruedas[0] = 60;
				ruedas[1] = 60;
			} else{ //Si no, giro a la izquierda
				ruedas[0] = 20;
				ruedas[1] = 140/2;
			}
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
		double[] delanteros = {todos[0], todos[1], todos[2], todos[3], todos[4], todos[5], todos[6]};
		double max = Funciones.maximo(delanteros); //Panel frontal
		
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
		for(int i=1; i<valores.length-1; i++){
			valores[i] = s.getValorSonar(i);
		}
		double[] delanteros = {valores[1], valores[2], valores[3], valores[4], valores[5], valores[6]};
		double min = Funciones.minimo(delanteros);

		if(min>this.UMBRAL)
			exito = true;
		
		return exito;
	}
	
	
	
	
}
