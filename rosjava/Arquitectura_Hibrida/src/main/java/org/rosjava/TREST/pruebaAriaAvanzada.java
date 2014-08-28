package org.rosjava.TREST;
import java.util.LinkedList;

import planificador.gestorPlanes.Accion;
import planificador.gestorPlanes.Meta;
import planificador.gestorPlanes.Plan;
import secuenciador.interprete.Tarea;
import infraestructura.Actuador;
import infraestructura.Estado;
import infraestructura.Laser;
import infraestructura.Motor;
import infraestructura.Camara;
import infraestructura.Robot;
import infraestructura.Sensor;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import geometry_msgs.Twist;
import org.ros.rosjava_geometry.Vector3;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;



public class pruebaAriaAvanzada extends AbstractNodeMain {
	
	/* Nmero de sensores snar del robot */
    static final int NUM_SENSORES = 2;
    
    /* Nmero de bumpers delanteros del robot */
 //   static final int NUM_BUMPERS = 5;
    
 
    public void onStart(final ConnectedNode node) {
/* Inicializacion del robot real y el servidor ARIA */
    
        
        /* Inicializacin de la arquitectura */
        Robot rob;
		Sensor[] sensores_robot = new Sensor[2];
		Actuador[] actuadores_robot = new Actuador[2];
		Estado estado = new Estado(null, new Tarea("Iniciar", null));
		Camara son = new Camara(1,node);
		sensores_robot[0] = son;
		Laser bump = new Laser(node);
		sensores_robot[1] = bump;
		Motor motor = new Motor(node);
		Motor motor2 = new Motor(node);
		actuadores_robot[0] = motor;
		actuadores_robot[1] = motor2;
		System.out.println("INICIA EL PROGRAMA");
		/* Inicializacion del robot con sus elementos y el plan */
        Plan plan;
		Meta meta_global = new Meta("EvitarChoque", null);
		LinkedList<Accion> acciones = new LinkedList<Accion>();
        acciones.add(new Accion("avanzar"));
                
		plan = new Plan("PLAN4", meta_global, acciones);
		rob = new Robot(estado, sensores_robot,actuadores_robot,node ,false);
                rob.getArquitectura().getSecuenciador().setPlan(plan);		              
                for(int x=0 ;x<rob.getArquitectura().getSecuenciador().getAgenda().getPlan().getAcciones().size();x++)
                 //System.out.println(" EL PLAN DICE: "+x+" "+rob.getArquitectura().getSecuenciador().getAgenda().getPlan().getAcciones().get(x).getNombre());
                System.out.println("INICIA EJECUCION");

                while(rob.getBumpers().getValor()==0 && rob.getEstado().getTarea().getNombre().compareTo("EvitarObstaculo")!=0){
			           // System.out.println(rob.getEstado().getTarea().getNombre()+" bumpers "+rob.getBumpers().getValor());
			rob.getMotor().publicador();			                          
            try {
    			Thread.sleep(1500);
    		} catch (InterruptedException e) {
    			// TODO Auto-generated catch block
    			e.printStackTrace();
    		}
		}
        rob.getMotor().setRuedas(0, 0);
        rob.getMotor().publicador();
		System.out.println("Salir del Sistema");    	
    }



	@Override
	public GraphName getDefaultNodeName() {
		 return GraphName.of("SEGUIR/Infraestructura");
	}
	
}

