#include "ros/ros.h"

#define filas 8
#define columnas 10

using namespace std;

int matriz[filas][columnas]={{1,1,1,1,1,1,1,1,1,1},
                             {1,0,0,0,0,0,0,0,0,1},
                             {1,0,0,0,0,0,0,0,0,1},
                             {1,0,0,0,0,0,0,0,0,1},
                             {1,0,0,0,1,1,0,0,0,1},
                             {1,0,0,0,1,1,0,0,0,1},
                             {1,0,0,0,1,1,0,0,0,1},
                             {1,1,1,1,1,1,1,1,1,1}};
int respuesta;
int columnainicial,columnafinal,filainicial,filafinal;
int columnainicial_,columnafinal_,filainicial_,filafinal_;
int signo=1;
double distancia;
int main(int argc, char **argv)
{
	ros::init(argc,argv,"detecta_obstaculos");
	ros::NodeHandle nodo;


while(ros::ok())
{
    for(int i=0;i<filas;i++)
    {
       for(int k=0;k<columnas;k++)
            {
             cout<<matriz[i][k]<<" ";
            }
       cout<<endl;
    }
cout<<"filainicial"<<endl;
cin>>filainicial;
cout<<"columnainicial"<<endl;
cin>>columnainicial;
cout<<"filafinal"<<endl;
cin>>filafinal;
cout<<"columnafinal"<<endl;
cin>>columnafinal;

filafinal_=filafinal;filainicial_=filainicial;columnafinal_=columnafinal_;columnainicial_=columnainicial;

if(columnafinal<columnainicial)
{
    filafinal=filainicial_;
    columnafinal=columnainicial_;
    columnainicial=columnafinal_;
    filainicial=filafinal_;
    cout<<"EH! el punto final está a la izquierda, vamos a cambiarnos"<<endl;
}
double theta=atan2((filafinal-filainicial),(columnafinal-columnainicial));
if(filafinal>=filainicial)
{

    signo=1;
    cout<<filafinal<<"mayor que"<<filainicial<<"el signo es :  "<<signo<<endl;
}
else if(filafinal<filainicial)
{
    signo=-1;
    cout<<"el signo es :  "<<signo<<endl;
}
cout<<"el angulo es :"<<theta<<endl;

bool obstaculo=false;
for(float i=columnainicial;i<columnafinal;i=i+0.2)
{    //cout<<"sumando"<<i-columnainicial<<endl;
    if(obstaculo==false)
    {
        distancia=(tan(theta)*(i-columnainicial));
        int altura=filainicial+0.5+distancia;
        //cout<<"sumando"<<filainicial<<"+0.5"<<distancia<<endl;
        int elementohorizontal = i+0.5;
        cout <<"comprobando el elemento de  la matriz :   ["<<altura<<","<<elementohorizontal<<"]"<<endl;
        int alturainferior = signo*altura-signo*0.5;
        int alturasuperior = signo*altura+signo*0.5;

        if(matriz[altura][elementohorizontal]==1)
        {
            obstaculo=true;
            cout<<"primer obstaculo encontrado en el ["<<elementohorizontal<<","<<altura<<"]"<<endl;
        }else if(matriz[alturainferior][elementohorizontal]==1)
        {
            obstaculo=true;
            cout<<"primer obstaculo encontrado con offset superior en el ["<<elementohorizontal<<","<<alturainferior<<"]"<<endl;
        }else if(matriz[alturasuperior][elementohorizontal]==1)
        {
            obstaculo=true;
            cout<<"primer obstaculo encontrado con offset inferior en el ["<<elementohorizontal<<","<<alturasuperior<<"]"<<endl;
        }else
            obstaculo = false;
    }
}

respuesta=obstaculo;
cout<<"hay obstaculo?:    "<<respuesta<<endl;


ros::spinOnce();
}
}

