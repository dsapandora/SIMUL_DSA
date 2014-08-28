package planificador.gestorPlanes;

import java.util.LinkedList;

/**
 * Clase que contiene la meta final del robot para un problema concreto 
 * y la secuencia ordenada de acciones que se deben llevar a cabo para 
 * alcanzar dicha meta. A partir de este plan, las capas inferiores 
 * trabajan para que el robot lo ejecute con exito.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Plan {
	
	/**
	 * Identificador ounico del plan; se utiliza la cadena "PLAN" seguida de un 
	 * noumero entero.
	 */
	private String id;
	
	/**
	 * Meta final que se desea alcanzar al desarrollar el plan por el robot.
	 */
	private Meta meta_global;
	
	/**
	 * Lista ordenada de acciones que debe llevar a cabo el robot para alcanzar 
	 * la meta impuesta para el plan.
	 */
	private LinkedList<Accion> acciones;
	
	
	/**
	 * Inicializa con los atributos a null.
	 */
	public Plan(){}
	
	/**
	 * Constructor por defecto para las pruebas del secuenciador, que necesitan 
	 * la lista de acciones. NO USAR PARA CREAR PLANES CON EL PLANIFICADOR.
	 */
	public Plan(String id, Meta meta, LinkedList<Accion> acc){
		this.id = id;
		this.meta_global = meta;
		this.acciones = acc;
	}
	
	/**
	 * Inicializa el plan, asignandole un identificador y un objetivo, para posteriormente 
	 * planificar la lista de acciones necesarias para alcanzarlo.
	 * 
	 * @param id El identificador ounico del plan
	 * @param meta El objetivo final del plan
	 */
	public Plan(String id, Meta meta){
		this.id = id;
		this.meta_global = meta;
		this.acciones = new LinkedList<Accion>();
	}
	
	
	/**
	 * Devuelve el identificador ounico del plan.
	 * 
	 * @return EL identificador del plan
	 */
	public String getId(){
		return this.id;
	}
	
	/**
	 * Devuelve el objetivo final del plan.
	 * 
	 * @return La meta asociada al plan
	 */
	public Meta getMeta(){
		return this.meta_global;
	}
	
	/**
	 * Devuelve la lista ordenada de acciones a llevar a cabo por el robot para 
	 * alcanzar el objetivo del plan.
	 * 
	 * @return La lista de acciones del plan
	 */
	public LinkedList<Accion> getAcciones(){
		return this.acciones;
	}
	
	/**
	 * Asigna un nuevo identificador ounico al plan.
	 * 
	 * @param id El nuevo identificador del plan
	 */
	public void setId(String id){
		this.id = id;
	}
	
	/**
	 * Asigna un nuevo objetivo final al plan.
	 * 
	 * @param meta La nueva meta asociada al plan
	 */
	public void setMeta(Meta meta){
		this.meta_global = meta;
	}
	
	/**
	 * Anade una accion en la oultima posicion a la lista ordenada de acciones 
	 * del plan.
	 * 
	 * @param accion La nueva accion a incluir en el plan
	 */
	public void anyadirAccion(Accion accion){
		this.acciones.add(accion);
	}
	
	/**
	 * Indica si un plan es igual a otro. Dos planes son iguales si sus listas de 
	 * acciones coinciden en tamano, y ademas las acciones son iguales y estan en 
	 * el mismo orden.
	 * 
	 * @param plan El plan con el que se desea comparar
	 * @return "true" si los dos planes son iguales, "false" en caso contrario
	 */
	public boolean esIgual(Plan plan){
		boolean igual = true;
		
		if(this.acciones.size()==plan.getAcciones().size()){
			int i = 0;
			while(i<this.acciones.size() && igual){
				String n1 = this.acciones.get(i).getNombre();
				String n2 = plan.getAcciones().get(i).getNombre();
				if(n1.compareTo(n2)!=0)
					igual = false;
				i++;
			}
		} else
			igual = false;
		
		return igual;
	}

}
