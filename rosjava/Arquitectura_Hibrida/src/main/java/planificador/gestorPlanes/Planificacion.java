package planificador.gestorPlanes;

import java.util.LinkedList;
import java.util.StringTokenizer;
import utilidades.FicheroLectura;

/**
 * Clase que se encarga de realizar la planificacion, en funcion 
 * del objetivo final del robot. Utiliza un algoritmo de busqueda 
 * para elaborar el plan, haciendo uso para ello de los elementos 
 * clasicos de planificacion: acciones y predicados.
 * 
 * @author Jose Luis Diaz Cebrian
 * @version 1.0
 * 
 */
public class Planificacion {
	
	/**
	 * Libreria con todos los tipos de elementos definidos en el sistema.
	 */
	private Elemento[] libreria_elementos;
	
	/**
	 * Libreria con todos los tipos de predicados definidos en el sistema.
	 */
	private LinkedList<Predicado> libreria_predicados;
	
	/**
	 * Libreria con todas las acciones definidas en el sistema.
	 */
	private LinkedList<Accion> libreria_acciones;
	
	/**
	 * Lista con todos los predicados instanciados en el momento actual, que definen 
	 * el estado del modelo del mundo en cada momento.
	 */
	private LinkedList<Predicado> estado_actual;
	
	
	/**
	 * Inicializa la instancia de la planificacion, asignando el estado actual del 
	 * modelo del mundo y creando las librerias de los objetos de planificacion a partir 
	 * de un fichero de configuracion del planificador.
	 * 
	 * @param fich El nombre del fichero de configuracion que define el dominio del planificador
	 * @param estado La lista de predicados con el estado actual del modelo del mundo
	 */
	public Planificacion(String fich, LinkedList<Predicado> estado){
		this.estado_actual = estado;
		this.libreria_predicados = new LinkedList<Predicado>();
		this.libreria_acciones = new LinkedList<Accion>();
		this.inicializar(fich);
	}
	
	
	/**
	 * Metodo auxiliar para la inicializacion de las librerias del planificador a partir de 
	 * la lectura del fichero de configuracion que define el dominio del planificador.
	 * 
	 * @param fich El nombre del fichero de configuracion del dominio
	 */
	private void inicializar(String fich){
		FicheroLectura fichero;
		fichero = new FicheroLectura(fich);
		String linea = "";
		
		fichero.abrir();
		
		/* Ejemplo de creacion de librerias a partir del ejemplo de fichero de config. */
		linea = fichero.leerLinea();
		/* Creacion de la libreria de elementos */
		if(linea.compareTo("#Elementos")==0){
			linea = fichero.leerLinea();
			StringTokenizer st = new StringTokenizer(linea, " ");
			int i = 0;
			this.libreria_elementos = new Elemento[st.countTokens()];
			while(st.hasMoreTokens()){
				String tipo = st.nextToken();
				this.libreria_elementos[i] = new Elemento(tipo, null);
				i++;
			}
			linea = fichero.leerLinea();
			/* Creacion de la libreria de elementos */
			if(linea.compareTo("#Predicados")==0){
				linea = fichero.leerLinea();
				while(linea.compareTo("#Acciones")!=0){
					st = new StringTokenizer(linea, " ");
					String nom = st.nextToken();
					Elemento[] elementos = new Elemento[st.countTokens()];
					i = 0;
					while(st.hasMoreTokens()){
						elementos[i] = new Elemento(st.nextToken(), null);
						i++;
					}
					this.libreria_predicados.add(new Predicado(nom, elementos));
					linea = fichero.leerLinea();
				}
				linea = fichero.leerLinea();
				Accion accion = new Accion();
				/* Creacion de la libreria de acciones */
				while(linea!=null){
					st = new StringTokenizer(linea, " ");
					String caso = st.nextToken();
					String nombre = st.nextToken();
					Elemento[] elementos = new Elemento[st.countTokens()];
					i = 0;
					if(caso.compareTo("@")==0)
						accion = new Accion();
					while(st.hasMoreTokens()){
						String token = st.nextToken();
						StringTokenizer otro = new StringTokenizer(token, ".");
						elementos[i] = new Elemento(otro.nextToken(), otro.nextToken());
						i++;
					}
					if(caso.compareTo("@")==0){
						accion.SetNombre(nombre);
						accion.setElementos(elementos);
					} else if(caso.compareTo(".")==0){
						accion.anyadirPrecondicion(new Predicado(nombre, elementos));
					} else if(caso.compareTo("-")==0){
						accion.anyadirBorrado(new Predicado(nombre, elementos));
					} else if(caso.compareTo("+")==0){
						accion.anyadirAnyadido(new Predicado(nombre, elementos));
					} else{
						System.out.println("ERROR - Formato de fichero de dominio incorrecto");
					}
					linea = fichero.leerLinea();
					if(linea.startsWith("@"))
						this.libreria_acciones.add(accion);
				}
			} else{
				System.out.println("ERROR - Formato de fichero de dominio incorrecto");
			}
		} else{
			System.out.println("ERROR - Formato de fichero de dominio incorrecto");
		}
		
		fichero.cerrar();
	}
	
	/**
	 * Elabora el plan que garantiza la consecucion de acciones que llevan a alcanzar la meta 
	 * global definida para el robot, haciendo uso de un algoritmo de busqueda.
	 * 
	 * @param meta_global La meta (objetivo) final que persigue el robot
	 * @param estado_inicial La lista de predicados que define el estado inicial del modelo del mundo
	 * @return El plan elaborado para alcanzar la meta global del robot
	 */
	public Plan planificar(Meta meta_global, LinkedList<Predicado> estado_inicial){
		Plan plan = new Plan();
		this.estado_actual = estado_inicial;
		
		/* TODO Implementar aqui el algoritmo de busqueda para la planificacion */
		
		return plan;
	}
	
	/**
	 * Elabora de nuevo un plan previamente planificado a partir del estado actual, en caso de 
	 * que el plan original haya fallado y el secuenciador no haya podido solventar el problema.
	 * 
	 * @param plan El plan original que ha producido el fallo
	 * @param submeta La submeta a partir de la cual el plan ha fallado
	 * @param estado_actual La lista de predicados que define el ewstado actual del modelo del mundo
	 * @return El nuevo plan elaborado a partir del original
	 * 
	 * TODO Metodo no terminado de implementar, solo incluido como ejemplo
	 */
	public Plan replanificar(Plan plan, Meta submeta, LinkedList<Predicado> estado_actual){
		this.estado_actual = estado_actual;

		/* TODO Implementar aqui el algoritmo de busqueda para la planificacion */
		
		return plan;
	}
	
	/**
	 * Busca un predicado instanciado concreto dentro del estado actual del modelo del mundo, 
	 * para comprobar si una precondicion de una accion se esta cumpliendo en el momento actual, 
	 * lo que permitiria incluirla en el plan que se esta elaborando.+
	 * 
	 * @param pred El predicado instanciado a buscar en el estado actual
	 * @return El indice dentro de la lista donde se encuentra el predicado. En caso de 
	 * no encontrarse, devolvera un indice no valido (una unidad por encima del tamano de 
	 * la lista).
	 * 
	 * TODO Metodo no terminado de implementar, solo incluido como ejemplo
	 */
	public int buscarPredicado(Predicado pred){
		int posicion = 0;
		boolean encontrado = false;
		
		while(posicion<this.estado_actual.size() && !encontrado){
			Predicado predicado = this.estado_actual.get(posicion);
			if(predicado.esIgual(pred))
				encontrado = true;
			else	
				posicion++;
		}
		
		return posicion;
	}
	
	/**
	 * Incluye una accion en el plan que se esta elaborando, previa comprobacion de que se 
	 * cumplen todas las precondiciones de la accion, buscandolas en el estado actual del 
	 * modelo del mundo. Se anade la accion al plan, se eliminan del estado actual aquellos 
	 * predicados que indique la accion, y se incluyen en el mismo aquellos predicados nuevos 
	 * resultantes de la ejecucion de la accion.
	 * 
	 * @param plan El plan actual que se esta elaborando
	 * @param accion La accion a incluir en el plan
	 * 
	 * TODO Metodo no terminado de implementar, solo incluido como ejemplo
	 */
	public void incluirAccion(Plan plan, Accion accion){
		/* Se borran del estado actual las postcondiciones correspondientes de la accion */
		LinkedList<Predicado> borrados = accion.getBorrados();
		for(int i=0; i<borrados.size(); i++){
			Predicado pred = borrados.get(i);
			/* TODO Terminar la implementacion considerando los elementos concretos de los predicados */
			this.estado_actual.remove(pred);			
		}
		/* Se anaden al estado actual las postcondiciones correspondientes de la accion */
		LinkedList<Predicado> anyadidos = accion.getAnyadidos();
		for(int i=0; i<anyadidos.size(); i++){
			Predicado pred = anyadidos.get(i);
			/* TODO Terminar la implementacion considerando los elementos concretos de los predicados */
			this.estado_actual.add(pred);			
		}
		/* Se incluye la accion en el plan */
		plan.anyadirAccion(accion);
	}


}
