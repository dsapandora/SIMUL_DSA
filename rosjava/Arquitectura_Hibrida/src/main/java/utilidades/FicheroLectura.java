package utilidades;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileNotFoundException;
import java.io.IOException;

/**
 * Clase auxiliar que representa un fichero tratado en modo lectura.
 * 
 * @author Josu Luis Duaz Cebriun
 * @version 1.0
 * 
 */
public class FicheroLectura{
    
    /**
     * Fichero abierto en modo lectura.
     */
    private BufferedReader _fichero;
    
    /**
     * Nombre del fichero.
     */
    private String _nombre;
    
    /**
     * Numero de lunea donde se encuentra la ultima lectura realizada.
     */
    private int _numLinea;
    
    
    /**
     * Constructor que recibe el nombre de un fichero para tratarlo en modo lectura.
     *  
     * @param nombreFichero Nombre del fichero a leer
     */
    public FicheroLectura(String nombreFichero){
        _nombre = new String(nombreFichero);
        _numLinea = 1;
    }
    
    
    /**
     * Mutodo que abre el fichero en modo lectura. Si no es posible, devuelve el valor 
     * booleano "false".
     * 
     * @return "true" si se abre el fichero correctamente, "false" en caso contrario
     */
    public boolean abrir(){
        boolean exito;
        exito = false;
        try{
            _fichero = new BufferedReader(new FileReader(_nombre));
            exito = true;
        }catch(FileNotFoundException e){
            System.out.println("Fichero " + _nombre + " no encontrado");
        }
        return exito;
    }
    
    /**
     * Mutodo que lee un caructer del fichero. En caso de llegar al final del fichero, devuelve 
     * un caructer en blanco.
     * 
     * @return El caructer leudo del fichero
     */
    public char leerCaracter(){
        int caracter;
        caracter = 0;
        try{
            caracter = _fichero.read();
        }catch(IOException e){
            System.out.println("Error en la lectura del fichero " + _nombre);
        }
        if(caracter == '\n'){
            incrementarNumLinea();
        }
        //Si es final de fichero
        if(caracter == -1){
            return ' ';
        } else{
            return (char)caracter;
        }
    }
    
    /**
     * Mutodo que lee una palabra. En caso de llegar al final del fichero, devuelve null.
     * 
     * @return La palabra leuda del fichero
     */
    public String leerPalabra(){
        String palabra;
        int caracter;
        palabra = new String();
        try{
            caracter = _fichero.read();
            while((caracter != ' ') && (caracter != '\n') && (caracter != '\t') && 
            (caracter != '\r') && (caracter != -1)){
                palabra = palabra + (char)caracter;
                caracter = _fichero.read();
            }
            if(caracter == '\n'){
                incrementarNumLinea();
            }
            if(caracter == -1){
                palabra = null;
            }
        }catch(IOException e){
            System.out.println("Error en la lectura del fichero " + _nombre);
        }
        return palabra;
    }
    
    /**
     * Mutodo que lee una lunea del fichero.
     * 
     * @return La lunea leuda del fichero
     */
    public String leerLinea(){
        String linea;
        linea = new String();
        try{
            linea = _fichero.readLine();
            incrementarNumLinea();
        }catch(IOException e){
            System.out.println("Error en la lectura del fichero " + _nombre);
        }
        return linea;
    }
    
    /**
     * Mutodo que cierra el fichero.
     * 
     * @return "true" si se cierra el fichero correctamente, "false" en caso contrario
     */
    public boolean cerrar(){
        boolean exito;
        exito = false;
        try{
            _fichero.close();
            exito = true;
        }catch(IOException e){
            System.out.println("Error al cerrar el fichero " + _nombre);
        }
        return exito;
    }
    
    /**
     * Mutodo que incrementa en una unidad el numero de lunea.
     */
    public void incrementarNumLinea(){
        _numLinea++;
    }
    
    /**
     * Mutodo que devuelve el numero de lunea actual.
     * 
     * @return El numero de lunea por la que se va leyendo
     */
    public  int obtenerNumLinea(){
        return _numLinea;
    }
    
}

