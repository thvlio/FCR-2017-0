// bibliotecas do C++
#include <iostream> //
#include <cmath> //
#include <iomanip> //
#include <vector>
#include <string>
#include <fstream> //
#include <sstream> //

// bibliotecas do ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <p2os_msgs/SonarArray.h>
#include <nav_msgs/Odometry.h>	//
#include <tf/tf.h> //
#include <tf/transform_datatypes.h> //
#include <fcr2017/my_classes.h> //

class ObstacleAvoidance {
	public:
		ObstacleAvoidance(ros::NodeHandle nh);
		void spin();
		~ObstacleAvoidance();

	private:
		ros::NodeHandle nh_;
		ros::Publisher vel_pub_;
		ros::Publisher grid_pub_; //
		ros::Subscriber laser_sub_, sonar_sub_, dsr_sub_;
		ros::Subscriber odom_sub_; //
		ros::Subscriber pos_sub_; //

		geometry_msgs::Twist command_vel_, desired_vel_;
		sensor_msgs::LaserScan scan_msg_;
		p2os_msgs::SonarArray sonar_msg_;
		nav_msgs::Odometry odom_msg_; //
		geometry_msgs::Point pos_msg_; //
		std_msgs::Float64MultiArray grid_msg_; //
		
		std::vector<Node> Graph_;	// vetor de nós usado para representar o grafo
		std::vector<Node> Sequence_;	// vetor de ponteiros de nó que definem a sequência que o robô deve seguir
		std::vector<Point> Path_; 	// vetor de pontos criado a partir da sequência de nós a ser seguida
		int currentNode_;	// nó atual do robô
		Point target_;	// configuração final especificado pelo usuário
		int targetNode_;	// nó alvo ao qual pertence o ponto target_

		int step_,		// contador que indica em que parte do trajeto o robô está
			finishedMapping_,	// flag que indica se o robô finalizou todo o mapeamento
			spin_,	// flag indica se o robô chegou no centro de um nó e deve olhar ao seu redor durante o mapeamento ou para indicar que o robô deve ajustar sua posição final depois que receber um comando de posição
			takeCommand_,	// flag que indica que o robô está pronto para receber um comando de posição
			finishedMoving_,	// flag que indica que o robô chegou ao alvo especificado pelo usuário
			receivedValidMsg_;	// indica se uma mensagem válida de posição foi recebida
		
		std::vector<Grid> Map_;	// vetor de grades que representa o mapa métrico interno do robô
		
		ros::Time callbackTime_;	// instante de tempo em que a função callback do laser é chamada

		void laserCallback (const sensor_msgs::LaserScan::ConstPtr& laser_msg);
		void sonarCallback (const p2os_msgs::SonarArray::ConstPtr& sonar_msg);
		void dsrCallback (const geometry_msgs::Twist::ConstPtr& desired_vel);
		void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg); //
		void posCallback (const geometry_msgs::Point::ConstPtr& pos_msg); //

		void algorithm ();
		
};
