package infraestructura;

/**
 * Clase abstracta que sirve como ejemplo de extensibilidad 
 * de la representaci�n del robot a la hora de a�adir nuevos 
 * elementos en el modelo del mundo. Ver manual de usuario.
 * 
 * @author Jos� Luis D�az Cebri�n
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
	 * TODO Incluir m�todos de acceso asociados a los atributos definidos.
	 * 
	 * @return El valor de retorno
	 * 
	 */
	public abstract int getNivel();
	
	/**
	 * TODO Incluir otros m�todos necesarios para la clase.
	 * 
	 * @param nivel Un par�metro del m�todo
	 */
	public abstract void setNivel(int nivel);

}
