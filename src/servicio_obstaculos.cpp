#include "ros/ros.h"
#include "planificacion_pkg/type_obstaculo.h"

using namespace std;


//~ void rellenarparametros(const std_msgs::UInt16MultiArray::ConstPtr& msg)
bool servicio(planificacion_pkg::type_obstaculo::Request &req, planificacion_pkg::type_obstaculo::Response &res)
{
	
	std_msgs::Int32 respuesta;
	int columnainicial,columnafinal,filainicial,filafinal;
	int columnainicial_,columnafinal_,filainicial_,filafinal_;
	int signo=1;
	double distancia;
	
	ROS_INFO("recibido mapa");

	int filas=req.mapa.layout.dim[0].size;
	int columnas=req.mapa.layout.dim[1].size;

	// Por comodidad, pasamos los datos guardados en el multiarray a una matriz de dos dimensiones
	int matriz[filas][columnas];
	for(int i=0;i<filas;i++)
	{
		for(int j=0;j<columnas;j++)
			matriz[i][j]=req.mapa.data[i*columnas+j];
	}
	
	//#################################################
	
	//Se muestra la matriz creada
	for(int i=0;i<filas;i++)
        {
           for(int k=0;k<columnas;k++)
                {
                 cout<<matriz[i][k]<<" ";
                }
           cout<<endl;
        }

        filainicial=req.nodoinicial.x;
        cout<<"filainicial = "<<filainicial<<endl;

        columnainicial=req.nodoinicial.y;
        cout<<"columnainicial = "<<columnainicial<<endl;

        filafinal=req.nodofinal.x;
        cout<<"filaifinal = "<<filafinal<<endl;

        columnafinal=req.nodofinal.y;
        cout<<"columnafinal="<<columnafinal<<endl;

        // El algoritmo está preparado para que funcione siempre empezando a valorar los puntos más a la derecha
        filafinal_=filafinal;
        filainicial_=filainicial;
        columnafinal_=columnafinal;
        columnainicial_=columnainicial;

        if(columnafinal<columnainicial)
        {
            filafinal=filainicial_;
            columnafinal=columnainicial_;
            columnainicial=columnafinal_;
            filainicial=filafinal_;
           /*cout<<"EH! el punto final está a la izquierda, vamos a cambiarnos"<<endl;
           cout<<"pasamos del ["<<filainicial_<<","<<columnainicial_<<"] al ["<<filainicial<<","<<columnainicial<<"]"<<endl;
           cout<<"pasamos del ["<<filafinal_<<","<<columnafinal_<<"] al ["<<filafinal<<","<<columnafinal<<"]"<<endl;*/
        }
        double theta = atan2((filafinal-filainicial),(columnafinal-columnainicial));
        if(filafinal>=filainicial)
        {

            signo=1;
//            cout<<filafinal<<"mayor que"<<filainicial<<"el signo es :  "<<signo<<endl;
        }
        else if(filafinal<filainicial)
        {
            signo=-1;
//            cout<<"el signo es : "<<signo<<endl;
        }
//        cout<<"el angulo es:"<<theta<<endl;

        bool obstaculo=false;

        if(columnainicial!=columnafinal)
        {
            for(float i=columnainicial;i<columnafinal;i=i+0.1)
            {
//                cout<<"sumando"<<i-columnainicial<<endl;
                if(obstaculo==false)
                {
                    distancia=(tan(theta)*(i-columnainicial));
                    float mostraraltura=filainicial+0.5+distancia;
                    int altura= mostraraltura;
                    //cout<<"la altura es:  "<<filainicial<<"+0.5+"<<distancia<<"="<<mostraraltura<<endl;
                    float mostrarelementohorizontal = i+0.5;
                    int elementohorizontal= mostrarelementohorizontal;
//                    cout <<"comprobando el elemento de  la matriz :   ["<<mostraraltura<<","<<mostrarelementohorizontal<<"]"<<endl;
                    //float mostraralturainferior = signo*mostraraltura+0.5-signo*0.25;
                    float mostraralturainferior = mostraraltura-signo*0.25;
                    int alturainferior= mostraralturainferior;
                    //float mostraralturasuperior = signo*mostraraltura+0.5+signo*0.25;
                    float mostraralturasuperior = mostraraltura+signo*0.25;
                    int alturasuperior= mostraralturasuperior;
                   //cout<<"comprobando el ["<<mostrarelementohorizontal<<","<<mostraraltura<<"]"<<endl;
                   //cout<<" offset inferior en el ["<<mostraralturainferior<<"="<<mostraraltura<<"-"<<signo<<"* 0.25"<<","<<mostrarelementohorizontal<<"]"<<endl;
                    //cout<<" offset superior en el ["<<mostraralturasuperior<<"="<<mostraraltura<<"+"<<signo<<"* 0.25"<<","<<mostrarelementohorizontal<<"]"<<endl;

                    if(matriz[altura][elementohorizontal]==1)
                    {
                        obstaculo=true;
                        cout<<"primer obstaculo encontrado en el ["<<altura<<","<<elementohorizontal<<"]"<<endl;
                        break;

                    }
                    else if(matriz[alturainferior][elementohorizontal]==1)
                    {
//                        cout<<"comprobando el ["<<elementohorizontal<<","<<alturainferior<<"="<<signo<<"*"<<altura<<"-"<<signo<<"*"<<"0.2"<<"]"<<endl;
                        obstaculo=true;
                        cout<<"primer obstaculo encontrado con offset superior en el ["<<alturainferior<<","<<elementohorizontal<<"]"<<endl;
                        break;
                    }
                    else if(matriz[alturasuperior][elementohorizontal]==1)
                    {
//                        cout<<"comprobando el ["<<elementohorizontal<<","<<alturasuperior<<"="<<signo<<"*"<<altura<<"+"<<signo<<"*"<<"0.2"<<"]"<<endl;
                        obstaculo=true;
                        cout<<"primer obstaculo encontrado con offset inferior en el ["<<alturasuperior<<","<<elementohorizontal<<"]"<<endl;
                        break;
                    }
                    else
                    {
                        obstaculo = false;
                       //cout<<"NO HAY OBSTACULO ALREDEDOR DEL ["<<elementohorizontal<<","<<altura<<"]"<<endl;

                    }
                }

            }
        }else
         {
              //cout<<"linea vertical entre fila "<<filainicial<<" y fila "<<filafinal<<endl;
              if(filafinal>filainicial)
              {
                  for(int k=filainicial;k<filafinal;k++)
                  {
                    //cout<<"comprobando elemento   ["<<k<<","<<columnainicial<<"]"<<endl;
                      if(matriz[k][columnainicial]==1)
                      {
                          obstaculo=true;
                          cout<<"Hay obstaculo en el elemento ["<<k<<","<<columnainicial<<"]"<<endl;
                          break;
                      }
                      else
                      {
                          cout<<"No hay obstaculo"<<endl;
                          obstaculo=false;
                      }
                  }
              }else
              {
                  //cout<<"meeeeh"<<endl;
                  cout<<"linea vertical entre fila "<<filainicial<<" y fila "<<filafinal<<endl;
                    for(int i=filainicial;i>filafinal;i=i-1)
                    {
                        cout<<"comprobando elemento   ["<<i<<","<<columnainicial<<"]"<<endl;
                        if(matriz[i][columnainicial]==1)
                        {
                            obstaculo=true;
                            cout<<"Hay obstaculo en el elemento ["<<i<<","<<columnainicial<<"]"<<endl;
                            break;
                        }
                        else
                        {
                            cout<<"No hay obstaculo"<<endl;
                            obstaculo=false;
                        }
                    }
              }
         }

        if(obstaculo==true)
			respuesta.data=1;
	    else
			respuesta.data=0;
        res.respuesta=respuesta;
        //~ cout<<"hay obstaculo?: "<<respuesta<<endl;
        return true;
	//##################################################
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"servicio_obstaculos");
    ros::NodeHandle nodo;
	//ros::Subscriber subscriptor = nodo.subscribe("mapa_topic",0, rellenarparametros);
	ros::ServiceServer service = nodo.advertiseService("servicio_obstaculo",servicio);
	ROS_INFO("Servicioregistrado");
	ros::spin();
	return 0;
}


