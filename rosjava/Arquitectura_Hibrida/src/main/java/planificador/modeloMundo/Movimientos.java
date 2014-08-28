package planificador.modeloMundo;

import utilidades.FicheroLectura;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.StringTokenizer;

/**
 * Clase que representa el mapa del modelo mundo, por el que el robot 
 * se mueve. Se trata de un mapa de movimientos, en el que se representan 
 * diferentes localizaciones concretas (nodos) y las conexiones entre ustas 
 * mediante arcos, que definene estrategias de movimientos. Sirve como apoyo 
 * a la planificaciun para la identificaciun de submetas en busca de la meta 
 * global del robot. Contiene la informaciun sobre los elementos relevantes 
 * identificados para el problema, tales como la posiciun del robot, de la 
 * meta o de la posible base de recarga de la baterua.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 *
 */
public class Movimientos extends Mapa{
	
	/**
	 * Lista de todas las localizaciones (nodos) que forman el mapa del modelo 
	 * del mundo.
	 */
	private Map<String, Nodo> lista_nodos;
	
	/**
	 * Lista de todas las conexiones (arcos) que forman el mapa del modelo del 
	 * mundo.
	 */
	private LinkedList<Arco> lista_arcos;
	
	
	/**
	 * Inicializa el mapa del mundo a partir de un fichero de configuraciun.
	 * 
	 * @param fich El nombre del fichero de configuraciun que describe el mapa
	 */
	public Movimientos(String fich){
		super(fich);
		
		this.lista_nodos = new HashMap<String, Nodo>();
		
		FicheroLectura fichero;
		fichero = super.getFichero();
		
		String linea = "";
		
		fichero.abrir();
		
		/* Ejemplo de inicializaciun de mapa a travus del ejemplo del fichero de config. */
		linea = fichero.leerLinea();
		if(linea.compareTo("#Nodos")==0){
			linea = fichero.leerLinea();
			StringTokenizer st = new StringTokenizer(linea, " ");
			int i = 0;
			while(st.hasMoreTokens()){
				String n = st.nextToken();
				Nodo nodo = new Nodo(n);
				this.anyadirNodo(nodo);
				i++;
			}
			linea = fichero.leerLinea();
			if(linea.compareTo("#Arcos")==0){
				linea = fichero.leerLinea();
				while(linea!=null){
					st = new StringTokenizer(linea, " ");
					String n1 = st.nextToken();
					String n2 = st.nextToken();
					String estrategia = st.nextToken();
					Nodo nodo1 = this.lista_nodos.get(n1);
					Nodo nodo2 = this.lista_nodos.get(n2);
					Arco arco = new Arco(nodo1, nodo2, estrategia);
					this.anyadirArco(arco);
					nodo1.anyadirConexion(arco);
					nodo2.anyadirConexion(arco);
					linea = fichero.leerLinea();
				}
			} else{
				System.out.println("ERROR - Formato de fichero de mapa incorrecto");
			}
		} else{
			System.out.println("ERROR - Formato de fichero de mapa incorrecto");
		}
		
		fichero.cerrar();
	}
	
	
	/**
	 * Devuelve la lista de todas las localizaciones que forman el mapa del mundo.
	 * 
	 * @return La lista de nodos del mapa
	 */
	public Map<String, Nodo> getNodos(){
		return this.lista_nodos;
	}
	
	/**
	 * Devuelve una localizaciun concreta del mapa.
	 * 
	 * @param id El identificador de la localizaciun
	 * @return El nodo cuyo identificador coincide con el parumetro
	 */
	public Nodo getNodo(String id){
		return this.lista_nodos.get(id);
	}
	
	/**
	 * Devuelve la lista de todas las conexiones que forman el mapa del mundo.
	 * 
	 * @return La lista de arcos del mapa
	 */
	public LinkedList<Arco> getArcos(){
		return this.lista_arcos;
	}
	
	/**
	 * Auade una nueva localizaciun a la lista de nodos del mapa.
	 * 
	 * @param nodo El nodo a auadir a la lista
	 */
	public void anyadirNodo(Nodo nodo){
		this.lista_nodos.put(nodo.getId(), nodo);
	}
	
	/**
	 * Auade una nueva conexiun a la lista de arcos del mapa.
	 * 
	 * @param arco El arco a auadir a la lista
	 */
	public void anyadirArco(Arco arco){
		this.lista_arcos.add(arco);
	}
	

}
