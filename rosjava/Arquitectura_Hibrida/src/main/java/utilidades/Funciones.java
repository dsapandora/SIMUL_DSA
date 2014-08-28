package utilidades;

/**
 * Clase de funciones auxiliares, por ejemplo, matemuticas, para el 
 * uso del resto del sistema.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Funciones {
	
	/**
	 * Mutodo auxiliar que devuelve el munimo valor de un array de valores double.
	 * 
	 * @param valores El array de valores
	 * @return El munimo valor del array
	 */
	public static double minimo(double[] valores){
		double min = valores[0];
		for(int i=1; i<valores.length; i++){
			if(valores[i]<min)
				min = valores[i];
		}
		return min;
	}
	
	/**
	 * Mutodo auxiliar que devuelve el maximo valor de un array de valores double.
	 * 
	 * @param valores El array de valores
	 * @return El maximo valor del array
	 */
	public static double maximo(double[] valores){
		double max = valores[0];
		for(int i=1; i<valores.length; i++){
			if(valores[i]>max)
				max = valores[i];
		}
		return max;
	}

}
