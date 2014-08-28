package secuenciador.interprete;

import infraestructura.Estado;

import java.util.Collection;
import java.util.LinkedList;

import planificador.gestorPlanes.Accion;
import planificador.gestorPlanes.Meta;
import planificador.gestorPlanes.Metas;
import planificador.gestorPlanes.Plan;
import planificador.modeloMundo.Nodo;
import secuenciador.memoria.Historial;
import secuenciador.memoria.LibreriaRAPs;
import secuenciador.memoria.RAP;

/**
 * Clase que realiza la secuenciaciun de tareas, asignando un 
 * RAP a cada una de ellas y seleccionando posteriormente una 
 * habilidad concreta de ese RAP. Se encarga de la activaciun y 
 * desactivaciun de conjuntos de habilidades y de mandar urdenes 
 * al controlador.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Secuenciacion {

	/**
	 * Historial que almacena todos los planes resueltos desde que se iniciu el sistema, 
	 * con sus respectivas secuencias de tareas y RAPs utilizados.
	 */
	private Historial historial;
	
	/**
	 * Librerua con todos los RAPs definidos en el sistema.
	 */
	private LibreriaRAPs libreria_raps;
	
	/**
	 * Librerua con todas las tareas definidas en el sistema.
	 */
	private Tareas libreria_tareas;
	
	/**
	 * Librerua con todas las metas definidas en el sistema.
	 */
	private Metas libreria_metas;
	
	/**
	 * Plan a secuenciar para poder ser llevado a cabo por el robot del sistema.
	 */
	private Plan plan;
	
	/**
	 * Lista ordenada de RAPs utilizados para cada tarea incluida en la secuencia.
	 */
	private LinkedList<RAP> solucion;
	
	/**
	 * Lista ordenada de tareas que permiten llevar a cabo el plan elaborado.
	 */
	private LinkedList<Tarea> secuencia;
	
	
	/**
	 * Inicializa la instancia de la secuenciaciun creando el historial y rellenando 
	 * las diferentes libreruas del sistema.
	 */
	public Secuenciacion(){
		this.historial = new Historial();
		this.libreria_metas = new Metas();
		this.libreria_tareas = new Tareas(this.libreria_metas);
		this.libreria_raps = new LibreriaRAPs(this.libreria_metas);
		this.plan = null;
		this.solucion = new LinkedList<RAP>();
		this.secuencia = new LinkedList<Tarea>();
	}
	
	
	/**
	 * Devuelve el historial de planes almacenados en el sistema.
	 * 
	 * @return El historial del planes
	 */
	public Historial getHistorial(){
		return this.historial;
	}
	
	/**
	 * Devuelve la lista ordenada de RAPs obtenida para llevar a cabo del plan
	 * 
	 * @return La lista de RAPs a utilizar
	 */
	public LinkedList<RAP> getSolucion(){
		return this.solucion;
	}
	
	/**
	 * Devuelve la lista ordenada de tareas generada para llevar a cabo el plan
	 * 
	 * @return La lista de tareas a realizar
	 */
	public LinkedList<Tarea> getSecuencia(){
		return this.secuencia;
	}
	
	/**
	 * Guarda una nueva referencia al plan elaborado en caso de haber sido replanificado para 
	 * secuenciarlo de nuevo.
	 * 
	 * @param plan El nuevo plan a secuenciar
	 */
	public void actualizarPlan(Plan plan){
		this.plan = plan;
	}
	
	/**
	 * Mutodo que crea una secuencia de tareas para el plan establecido. Si el plan es 
	 * igual a uno secuenciado previamente, se reutiliza dicha soluciun. En caso contrario, 
	 * se crea una nueva secuencia de tareas desde cero.
	 */
	public void secuenciar(){
		
		/* 
		 * Por el momento, cada acciun estu asociada a una tarea, por lo que de cada acciun 
		 * se saca una tarea, y a usta se le asigna un RAP concreto. Luego se auade esta 
		 * informaciun a la secuencia. Esto puede modificarse cuando se implemente la 
		 * capa superior de la arquitectura.
		 */
		
		/* Se busca un plan igual en el historial de planes */
		boolean otro_igual = false;
		int i = 0;
		Plan otro = null;
		while(i<this.historial.getPlanes().size() && !otro_igual){
			otro = this.historial.getPlanes().get(i);
			if(this.plan.esIgual(otro))
				otro_igual = true;
		}
		if(otro_igual){ /* Si hay un plan igual al actual, se utiliza esa soluciun */
			this.secuencia = this.historial.getSecuencia(otro);
			this.solucion = this.historial.getSolucion(otro);
		} else{
			/* Si no, se elabora la secuencia */
			LinkedList<Accion> acciones = this.plan.getAcciones();
			int j = 0;
			while(j<acciones.size()){
				Accion siguiente = acciones.get(j);
				Tarea tarea = asignarTarea(siguiente);
				this.secuencia.add(tarea);
				asignarRAP(tarea);
				j++;
			}
		}
		
	}
	
	/**
	 * Mutodo de apoyo para la secuenciaciun de tareas. Asigna una tarea a una de las 
	 * acciones que forman el plan establecido. Esta tarea pasa a formar parte de la 
	 * secuencia de tareas.
	 * 
	 * @param accion La acciun del plan
	 * @return La tarea asignada a dicha acciun
	 */
	public Tarea asignarTarea(Accion accion){
		
		String nombre = accion.getNombre();
		Collection<Tarea> col = new LinkedList<Tarea>();
		col = libreria_tareas.getTareas().values();
		Tarea[] tareas = col.toArray(new Tarea[col.size()]);
		boolean encontrada = false;
		int i=0;
		Tarea tarea = tareas[0];

		while(i<tareas.length && !encontrada){
			tarea = (Tarea) tareas[i];
			if(nombre.compareTo(tarea.getNombre())==0){
				encontrada = true;
			}
			i++;
		}
		
		return tarea;
	}
	
	/**
	 * Mutodo de apoyo para la secuenciaciun de tareas. Asigna un RAP a una tarea concreta 
	 * ya incluida en la secuencia.
	 * 
	 * @param tarea La tarea a la que asignar el RAP
	 */
	public void asignarRAP(Tarea tarea){
		
		Meta meta = tarea.getMeta();
		Collection<RAP> col = new LinkedList<RAP>();
		col = libreria_raps.getLista().values();
		RAP[] raps = col.toArray(new RAP[col.size()]);
		boolean encontrado = false;
		int i=0;
		RAP rap = raps[0];
		
		while(i<raps.length && !encontrado){
			rap = raps[i];
			if(meta.esIgual(rap.getMeta())){
				encontrado = true;
			}
			i++;
		}

		this.solucion.add(rap);
	}
	
	/**
	 * Mutodo de apoyo a la secuenciaciun de tareas. Se encarga de seleccionar una 
	 * habilidad de entre todas las que forman un RAP concreto. La selecciun se basa 
	 * en el estado y la posiciun actual del robot.
	 * 
	 * @param rap El RAP que contiene las habilidades aptas para la tarea
	 * @param estado El estado actual del robot
	 * @param posicion La posiciun actual del robot dentro del mapa del mundo
	 * @return El identificador de la habilidad seleccionada
	 */
	public String seleccionarHabilidad(RAP rap, Estado estado, Nodo posicion){
		/* 
		 * Por el momento, se selecciona la primera habilidad del RAP, que es la mus adecuada 
		 * para la tarea. Esto puede modificarse cuando se implemente la capa superior de la 
		 * arquitectura para seleccionar otras habilidades en funciun del estado y la posiciun 
		 * del robot.
		 */
		return rap.getHabilidades().element();
	}
	
	
}
