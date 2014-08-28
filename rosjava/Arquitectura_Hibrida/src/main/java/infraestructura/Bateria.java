package infraestructura;

/**
 * Clase abstracta que sirve como ejemplo de extensibilidad 
 * de la representación del robot a la hora de añadir nuevos 
 * elementos en el modelo del mundo. Ver manual de usuario.
 * 
 * @author José Luis Díaz Cebrián
 * @version 1.0
 * 
 */
public abstract class Bateria {
	
	/**
	 * Posible atributo del nuevo elemento del robot: nivel.
	 * 
	 * TODO Incluir los atributos necesarios para representar el nuevo elemento del robot.
	 * 
	 */
	
	
	/**
	 * TODO Incluir el constructor correspondiente para inicializar el nuevo elemento del robot.
	 */
	public Bateria(){}
	
	
	/**
	 * TODO Incluir métodos de acceso asociados a los atributos definidos.
	 * 
	 * @return El valor de retorno
	 * 
	 */
	public abstract int getNivel();
	
	/**
	 * TODO Incluir otros métodos necesarios para la clase.
	 * 
	 * @param nivel Un parámetro del método
	 */
	public abstract void setNivel(int nivel);

}
