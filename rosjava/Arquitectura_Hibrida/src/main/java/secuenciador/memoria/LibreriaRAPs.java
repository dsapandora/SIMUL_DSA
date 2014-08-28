package secuenciador.memoria;

import java.util.HashMap;
import java.util.Map;

import planificador.gestorPlanes.Metas;

/**
 * Agregado de RAPs, se trata de la lista que incluye todos 
 * los RAPs definidos en el sistema para la secuenciaciun de 
 * tareas. Esta clase los engloba y proporciona el acceso a 
 * ellos.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class LibreriaRAPs {

	/**
	 * Conjunto de RAPs que forman la instancia del agregado. Se identifican dentro del conjunto
	 * por el identificador del RAP definido en @see RAP.
	 */
	private Map<String, RAP> lista;
	
	
	/**
	 * Inicializa la librerua de RAPs del sistema, creando una instancia de cada uno de los 
	 * RAPs que se utilizarun, a partir de la librerua de metas del sistema, haciendo por tanto 
	 * una relaciun uno a uno entre metas y RAPs.
	 * 
	 * @param libreria Librerua de todas las metas posibles permitidas en el sistema
	 */
	public LibreriaRAPs(Metas libreria){
		this.lista = new HashMap<String, RAP>();
		
		String hab1[] = {"Girar", "Avanzar"};
		this.lista.put("RAP1", new RAP("RAP1", libreria.getMeta("PuntoEnfrente"), hab1));
		
		String hab2[] = {"Avanzar", "Girar", "IrA", "SeguirPared"};
		this.lista.put("RAP2", new RAP("RAP2", libreria.getMeta("LlegarA"), hab2));
		
		String hab3[] = {"SeguirPared", "Avanzar"};
		this.lista.put("RAP3", new RAP("RAP3", libreria.getMeta("DetectarAlgo"), hab3));
		
		String hab4[] = {"Recuperarse", "EvitarObstaculo"};
		this.lista.put("RAP4", new RAP("RAP4", libreria.getMeta("SuperarChoque"), hab4));
		
		String hab5[] = {"Avanzar", "Girar", "EvitarObstaculo"};
		this.lista.put("RAP5", new RAP("RAP5", libreria.getMeta("MismaDistancia"), hab5));
		
		String hab6[] = {"EvitarObstaculo", "Girar", "Avanzar"};
		this.lista.put("RAP6", new RAP("RAP6", libreria.getMeta("EvitarChoque"), hab6));
		
		String hab7[] = {"CubrirDistancia", "Girar"};
		this.lista.put("RAP7", new RAP("RAP7", libreria.getMeta("DistanciaRecorrida"), hab7));
	}
	
	
	/**
	 * Devuelve la librerua completa de RAPs del sistema.
	 * 
	 * @return La lista de todos los RAPs
	 */
	public Map<String, RAP> getLista(){
		return this.lista;
	}
	
	/**
	 * Devuelve un RAP concreto contenido en la librerua.
	 * 
	 * @param id El identificador unico del RAP
	 * @return EL RAP cuyo identificador coincide con dicho parumetro
	 */
	public RAP getRAP(String id){
		RAP rap = this.lista.get(id);
		if(rap==null)
			System.out.println("ERROR: No existe el RAP con identificador " + id);
		return rap;
	}
	
	/**
	 * Auade un nuevo RAP a la librerua de RAPs del sistema.
	 * 
	 * @param rap El nuevo RAP a incluir en la lista
	 */
	public void anyadirRAP(RAP rap){
		this.lista.put(rap.getId(), rap);
	}
	
	
}
