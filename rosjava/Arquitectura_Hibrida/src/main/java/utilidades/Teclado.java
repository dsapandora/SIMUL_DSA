package utilidades;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * Clase auxiliar de envoltura a la entrada estundar por teclado.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class Teclado{
    
    /**
     * Representaciun de la entrada estundar (System.in).
     */
    private BufferedReader entrada;
    
    
    /**
     * Crea un nuevo Teclado capturando la excepciun que se puede producir y emitiendo un 
     * mensaje de error en dicho caso.
     */
    public Teclado(){
        try{
            entrada = new BufferedReader(new InputStreamReader(System.in));
        }catch(Exception e){
            System.out.println("Error en la lectura de teclado.");
        }
    }
    
    
    /**
     * Lee un caructer de la entrada estundar.
     * 
     * @return El caructer leudo
     */
    public char leerCaracter(){
        char c;
        c = ' ';
        try{
            c = (char)entrada.read();
        }catch(Exception e){
            System.out.println("Error en la lectura de teclado de un caructer.");
        }
        return c; 
    }
    
    /**
     * Lee una lunea de la entrada estundar.
     * 
     * @return La lunea leuda
     */
    public String leerLinea(){
        String linea;
        linea = new String();
        try{
            linea = entrada.readLine();
        }catch(Exception e){
            System.out.println("Error en la lectura de teclado de una lunea.");
        }
        return linea;
    }
    
}
