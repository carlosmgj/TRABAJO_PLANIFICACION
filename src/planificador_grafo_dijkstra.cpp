#include <ros/ros.h>
#include "planificacion_pkg/lectura_mapa.h"
#include "planificacion_pkg/modificar_mapa.h"
#include "planificacion_pkg/type_obstaculo.h"
#include "planificacion_pkg/path.h"
#include <iostream>
#include <vector>

using namespace std;

class grafo1
{
public:
  grafo1()  {
    //Topic you want to publish
    pub_ = nodo_.advertise<planificacion_pkg::path>("camino_offset",0);
    pubp_ = nodo_.advertise<std_msgs::UInt16MultiArray>("data_mapa",0);
    //Topic you want to subscribe
    sub_ = nodo_.subscribe("goal_point",0, &grafo1::crear_plan, this);
    //serv
    serv_ = nodo_.serviceClient<planificacion_pkg::lectura_mapa>("leer_mapa");
    servp_ = nodo_.serviceClient<planificacion_pkg::modificar_mapa>("esquinas_mapa");
    servobs_ = nodo_.serviceClient<planificacion_pkg::type_obstaculo>("servicio_obstaculo");
    
    lectura_mapa();
    crear_matriz_relaciones();
  }

void lectura_mapa(){
  planificacion_pkg::lectura_mapa srv_leer;
  
  string nombre_mapa;
  if (ros::param::get("/map_name", nombre_mapa)) srv_leer.request.map_name=nombre_mapa;
  else srv_leer.request.map_name = "map1";

  if (serv_.call(srv_leer))
  {
	mapa = srv_leer.response.mapa;
	ROS_INFO("Nodo creado. A la espera de datos");
	size = mapa.data.size();
	columnas = mapa.layout.dim[1].size;
        cout<<"columnas=  "<<endl;
	filas=mapa.layout.dim[0].size;
        cout<<"filas=  "<<filas<<endl;
        int matriz[filas][columnas];

	//A la espera de saber como vamos a enviar el dato de punto inicial, marco el punto inicial del map1
  	pos_inicial = srv_leer.response.inicio.x*columnas + srv_leer.response.inicio.y;
    elementoinicial_x=srv_leer.response.inicio.x;
    elementoinicial_y=srv_leer.response.inicio.y;
    cout<<"posicion inicial  :"<<elementoinicial_x<<","<<elementoinicial_y<<"  (x,y)"<<endl;

    for(int i=0;i<filas;i++)
        {
                for(int j=0;j<columnas;j++)
                {
                     matriz[i][j]=mapa.data[i*columnas+j];
                }
        }



        //#################################################

   for(int i=0;i<filas;i++)
        {
           for(int k=0;k<columnas;k++)
                {
                 cout<<matriz[i][k]<<" ";
                 cout<<"b"<<endl;

                }
           cout<<endl;
        }
	//int a; cin >> a;
	pubp_.publish(mapa);
	ROS_INFO("Publicado");
  }
  else ROS_INFO("Error al llamar al servicio  de leer mapa");
     //#################################################
     
     
  planificacion_pkg::modificar_mapa srv_modificar;
  srv_modificar.request.inicial=mapa;
  srv_obstaculo.request.mapa=mapa;
  if (servp_.call(srv_modificar))
  {
		  mapaesquinas=srv_modificar.response.resultante;
		  geometry_msgs::Point elemento;
			int matrizesquinas[mapaesquinas.layout.dim[0].size][mapaesquinas.layout.dim[1].size];
			for(int k=0;k<mapaesquinas.layout.dim[0].size;k++)
			{
				for(int l=0;l<mapaesquinas.layout.dim[1].size;l++)
				{
					matrizesquinas[k][l]=mapaesquinas.data[k*mapaesquinas.layout.dim[1].size+l];
				}
			}
			for(int f=0;f<mapaesquinas.layout.dim[0].size;f++)
			{
				 for(int c=0;c<mapaesquinas.layout.dim[1].size;c++)
				 {
					 if(matrizesquinas[f][c]==2)
					 {
						elemento.x=f;
						elemento.y=c;
						vectoresquinas.push_back(elemento);
						cout<<"Esquinas en (x=columna)(y=fila):"<<elemento<<endl;
					 }
				 }
			}
			cout<<"Esquinas en (x=columna)(y=fila):";
			for(int v=0;v<vectoresquinas.size();v++)
			{
                                cout<<"["<<vectoresquinas[v].x<<","<<vectoresquinas[v].y<<"]";
			}
			cout<<endl;

			ROS_INFO("bandera activada");
   }else ROS_INFO("Error al llamar al servicio de esquinas");
}

void crear_plan(const geometry_msgs::Point::ConstPtr& msg){

  std_msgs::Int32 auxrespuesta;
  //Marcamos el punto final
  pos_final = msg->x*columnas + msg->y;
  elementofinal_x=msg->x;
  elementofinal_y=msg->y;
  cout<<"posicion final  :"<<elementofinal_x<<","<<elementofinal_y<<"  (x,y)"<<endl;
  if (pos_final>=size && msg->y < columnas) { if (mapa.data[pos_final] == 1) ROS_INFO("El punto no es valido");}
  else
  {

  ROS_INFO("Calculando trayectoria para llegar al punto (%i, %i)", int(msg->x), int(msg->y));

  //Se inicializan todos los vectores creados
  inicializar_datos();

  
  //Calcular el camino a seguir
  calcular_camino();
  publicar();
  }
}

void inicializar_datos()
{
  camino.punto.clear();
  camino.punto.resize(0);
  path.resize(size);
  costes.resize(size);
  checked.resize(size);
  for (int i = 0; i<size; i++)
  {			
	if (mapa.data[i] == 1) checked[i] = 1;
	else checked[i] = false;
	costes[i] = 0;
	path[i] = 0;
  }
}

void crear_matriz_relaciones()
{
  std_msgs::Int32 auxrespuesta;

	for(int i=0;i<vectoresquinas.size();i++)
	{
		m_r.push_back(vector<float>());
		for(int j=0;j<vectoresquinas.size();j++)
		{
			//si esquinas[i]=esquinas[j] es -1
			//si no : si no obstaculos entre esquinas[i] y esquinas[j]: distancia(esquina[i],esquina[j])
			if(i==j) m_r[i].push_back(-1);
			else
			{
				srv_obstaculo.request.nodoinicial.x=vectoresquinas[i].x;
				srv_obstaculo.request.nodoinicial.y=vectoresquinas[i].y;
				srv_obstaculo.request.nodofinal.x=vectoresquinas[j].x;
				srv_obstaculo.request.nodofinal.y=vectoresquinas[j].y;
				//cout<<"llamando al servicio obstaculos desde (x,y):  ["<<vectoresquinas[i].x<<","<<vectoresquinas[i].y<<"]  hasta  ["<<vectoresquinas[j].x<<","<<vectoresquinas[j].y<<"]"<<endl;
				if(servobs_.call(srv_obstaculo))
				{
					auxrespuesta=srv_obstaculo.response.respuesta;
					if(auxrespuesta.data ==0)
					{
						cout<<"llamando al servicio obstaculos desde (x,y):  ["<<vectoresquinas[i].x<<","<<vectoresquinas[i].y<<"]  hasta  ["<<vectoresquinas[j].x<<","<<vectoresquinas[j].y<<"]"<<endl;
						ROS_INFO("Sin obstaculo");
						m_r[i].push_back(sqrt(pow((vectoresquinas[i].x-vectoresquinas[j].x),2)+pow((vectoresquinas[i].y-vectoresquinas[j].y),2)));
					}else
					{
						//ROS_INFO("Obstaculo encontrado");
						m_r[i].push_back(-1);
					}
				}else ROS_ERROR("Fallo al llamar al servicio: servicio_obstaculo");
			
			}
		}
	}
	/*DIbujar la matriz
	 for(int i=0;i<vectoresquinas.size();i++)
	{
		for(int j=0;j<vectoresquinas.size();j++)
		{
			cout << m_r[i][j] << " _ ";
		}
		cout << endl;
	}*/
}

geometry_msgs::Point data_to_point(int data, int c){

  geometry_msgs::Point punto;
  punto.y = data%c;
  punto.x = (data - punto.y) / c;
  punto.z = 0;

  return punto;
}

void calcular_camino()
{
  std_msgs::Int32 auxrespuesta;
  
	elemento.x=elementoinicial_x;
	elemento.y=elementoinicial_y;
	vectoresquinas.push_back(elemento);
	elemento.x=elementofinal_x;
	elemento.y=elementofinal_y;
	vectoresquinas.push_back(elemento);


			// Añadimos a matriz de relaciones los puntos inicial y final
			m_r.push_back(vector<float>());
			m_r.push_back(vector<float>());
           for(int i=vectoresquinas.size()-2;i<vectoresquinas.size();i++)
           {

                for(int j=0;j<vectoresquinas.size();j++)
                {
				     if (i < j) continue;
                     if(i==j)
                     {
                         m_r[i].push_back(-1);
                     }
                     else
                     {
                         srv_obstaculo.request.nodoinicial.x=vectoresquinas[i].x;
                         srv_obstaculo.request.nodoinicial.y=vectoresquinas[i].y;
                         srv_obstaculo.request.nodofinal.x=vectoresquinas[j].x;
                         srv_obstaculo.request.nodofinal.y=vectoresquinas[j].y;
                         cout<<"llamando al servicio obstaculos desde (x,y):  ["<<vectoresquinas[i].x<<","<<vectoresquinas[i].y<<"]  hasta  ["<<vectoresquinas[j].x<<","<<vectoresquinas[j].y<<"]"<<endl;
                         if(servobs_.call(srv_obstaculo))
                         {
                                 auxrespuesta=srv_obstaculo.response.respuesta;
                                 if(auxrespuesta.data ==0)
                                 {
                                ROS_INFO("Sin obstaculo");
                                
                                    m_r[i].push_back(sqrt(pow((vectoresquinas[i].x-vectoresquinas[j].x),2)+pow((vectoresquinas[i].y-vectoresquinas[j].y),2)));
									m_r[j].push_back(sqrt(pow((vectoresquinas[i].x-vectoresquinas[j].x),2)+pow((vectoresquinas[i].y-vectoresquinas[j].y),2)));
                                 }else
                                 {
                                     //ROS_INFO("Obstaculo encontrado");
                                     m_r[i].push_back(-1);
									 m_r[j].push_back(-1);
                                 }
                         }else
                         {
                                 ROS_ERROR("Fallo al llamar al servicio: servicio_obstaculo");
                                 
                         }
                     }
                }
            }

           /*Dibujar matriz de relaciones
            for(int i=0;i<vectoresquinas.size();i++)
            {
                for(int j=0;j<vectoresquinas.size();j++)
                {
                    cout<<m_r[i][j]<<" _ ";
                }
                cout<<endl;
            }*/
            
            //#################################PLANIFICADOR

            std::vector<bool> checked;
            std::vector<int> mejoresquina;
            std::vector<int> costeesquina;
            checked.resize(vectoresquinas.size());
            costeesquina.resize(vectoresquinas.size());
            mejoresquina.resize(vectoresquinas.size());
            for(int m=0;m<checked.size();m++)
            {
                checked[m]=false;
                mejoresquina[m]=0;
                costeesquina[m]=0;
            }
            int puntoacalcular=vectoresquinas.size()-1;

            while(puntoacalcular != vectoresquinas.size()-2)
            {   cout<<puntoacalcular<<endl;
                checked[puntoacalcular]=true;
                for(int columna=0;columna<vectoresquinas.size();columna++)
                   {
                        if(m_r[puntoacalcular][columna]==-1)
                            continue;
                        if(checked[columna]==true)
                            continue;
                        if(costeesquina[columna]==0)
                        {
                             costeesquina[columna]=m_r[puntoacalcular][columna] + costeesquina[puntoacalcular];
                             mejoresquina[columna]=puntoacalcular;
                        }
                        else if(m_r[puntoacalcular][columna] + costeesquina[puntoacalcular]<costeesquina[columna])
                        {
                            costeesquina[columna]=m_r[puntoacalcular][columna] + costeesquina[puntoacalcular];
                            mejoresquina[columna]=puntoacalcular;
                        }

                   }
                float minimocoste=1000;
                for(int columna=0;columna<vectoresquinas.size();columna++)
                   {
                    if(checked[columna]==true)
                        continue;
                    if(costeesquina[columna]==0)
                        continue;
                    if(costeesquina[columna]<minimocoste)
                    {
                        minimocoste=costeesquina[columna];
                        puntoacalcular=columna;
                    }
                    }

            }
           
           geometry_msgs::Point punto;
           int posicionesquina=vectoresquinas.size()-2;

           while(posicionesquina!=vectoresquinas.size()-1)
           {

               posicionesquina=mejoresquina[posicionesquina];

               punto.y=vectoresquinas[posicionesquina].y;
               punto.x=vectoresquinas[posicionesquina].x;
               cout<<"["<<punto.x<<","<<punto.y<<"]"<<endl;
               camino.punto.push_back(punto);
           }
}


void publicar() { 
	pub_.publish(camino); 
	pos_inicial = camino.punto[camino.punto.size()-1].x*columnas + camino.punto[camino.punto.size()-1].y;
}

private:
  ros::NodeHandle nodo_;
  ros::Publisher pub_;
  ros::Publisher pubp_;
  ros::Subscriber sub_;
  ros::ServiceClient serv_;
  ros::ServiceClient servp_;
  ros::ServiceClient servobs_;
  std_msgs::UInt16MultiArray mapa;
  std_msgs::UInt16MultiArray mapaesquinas;

  vector<int> path;
  vector<float> costes;
  vector<bool> checked;
  vector<geometry_msgs::Point> vectoresquinas;
  vector< vector<float> > m_r;
  
  int size;
  int filas;
  int columnas;
  int pos_inicial;
  int pos_final;
  int elementoinicial_x;
  int elementoinicial_y;
  int elementofinal_x;
  int elementofinal_y;
  
  planificacion_pkg::path camino;
  planificacion_pkg::type_obstaculo srv_obstaculo;
  
  
  geometry_msgs::Point elemento;
  
};//End of class


int main (int argc, char **argv)
{

  ros::init(argc, argv, "dijkstra");

  ROS_INFO("Creando nodo planificador");
  grafo1 planificador;

  ros::spin();



  return 0;
}
